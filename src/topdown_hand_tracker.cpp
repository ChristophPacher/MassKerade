/*!
 * @file	topdown_hand_tracker.cpp
 * @author  Christoph Pacher <chris@christophpacher.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2014 Christoph Pacher http://www.christophpacher.com
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 * the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and
 *   the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *   the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "topdown_hand_tracker.h"


#include <boost/date_time/posix_time/posix_time.hpp>
#include <ppl.h>


using namespace std;
using namespace cv;
using namespace pacher;
#ifdef PC_PERF_MARK
using namespace concurrency;
using namespace concurrency::diagnostic;
#endif // PC_PERF_MARK
using namespace boost::posix_time;


//#define HT_SHOW_DEBUG_IMG

std::mutex topdown_hand_tracker::mutex_{};

std::vector<person_result> topdown_hand_tracker::track( cv::Mat &depth_in, cv::Mat &rgb_in )
{
	
#ifdef PC_PERF_MARK	
	span span(*marker_series_, L"process frame");
#endif

	lock_guard<mutex> lock( mutex_ );

	upload_rgb( rgb_in );

	back_sub_.process( depth_in, depth_backremoved_, rgb_in, color_backremoved_);

	upload_depth();


	// complete last loop by moving new data into old datastore and reinit vars while images are uploaded to GPU

	contour_index_mask_ = Scalar( 0 );
	color_ortho_masked_ = Scalar( 0, 0, 0 );
	

	Mat depth_backremoved_8;
	auto transf = reprojector_.Transform_Z();
	depth_backremoved_.convertTo( depth_backremoved_8, CV_8UC1,  transf.mul, transf.add );

	// ortho reproject the images as a meshed and textured RGBD Pointcloud to get nice interpolated results without holes
	reprojector_.reproject( depth_backrem_tex_, rgb_tex_ );

	
	
	
	// we now have to spend some time on the cpu to give the gpu time to prepare the result

	// find features in the rgb image that are inside the mask 
	keypoints kps = feature_detector_.detect( rgb_in, depth_backremoved_8 );
	
	// ortho reproject the found features
	// some features can land outside of the image. 
	// they are removed in kps aswell as in kps_ortho
	keypoints kps_ortho = reprojector_.reproject( kps, depth_backremoved_ );


	// reset matched_ to false
	for ( auto &p : persons_)
	{
		p.matched_ = false;
	}

	const ptime current_time( microsec_clock::universal_time() );
	// purge the vector of disappeared person objects that are too old
	disappeared_.erase( remove_if( disappeared_.begin(), disappeared_.end(), [ &]( const person &p )
	{
		auto  dur = ( current_time - p.disappeared_time_ ).total_seconds();
		bool toDelete = dur > settings_.disappeared_timeout_s;
		if( toDelete )
		{
			cout << "id " << p.id_<< " removed from disappeared after " << dur << " seconds"  << endl;
		}
		return toDelete;
	} ), disappeared_.end() );


	// TODO pull gray conversion out of feature detector and pass grey into 
	//Mat gray;
	//cvtColor( rgb_in, gray, COLOR_BGR2GRAY );

	// after spending the time on cpu with something else, we get the reprojection result
	reprojector_.get_result( color_ortho_, depth_ortho_ );


	// blob.index_in_mask_ needs to be reduced by one, to be used as an index in blobs,
	// since gray value 0 is the background in 
	// contour_index_mask_ and the blob index has to start with 1
	// inside_blob() is already decrementing 
	vector<blob> blobs = blob_detector_.detect( depth_ortho_, contour_index_mask_ );
	
	// create a clean color ortho image with only the blobs inside
	color_ortho_.copyTo( color_ortho_masked_, contour_index_mask_ );
	
	// can only search in ortho image for local maxima since perspective would 
	// cause wrong results
	vector<local_maxima::PointValue> found_maxima = maxima_.detect( depth_ortho_, contour_index_mask_ );
	for ( const auto &m : found_maxima )
	{
		// copy found local maxima into corresponding blob
		const float blob_index = inside_blob( m.p.y, m.p.x );
		if ( blob_index > -1 )
		{
			blobs[ blob_index ].local_maxima_.push_back( m );
		}
	}
	for ( auto &b : blobs )
	{
		if( b.local_maxima_.size() == 0 )
			b.local_maxima_.emplace_back( b.pos_.x, b.pos_.y, 
			depth_ortho_.at<uchar>(b.pos_.y, b.pos_.x) );
	}
	

	// we use the blob centroid if there is no maxima
	// so we do not need to check if there is a maxima or not
	for (auto &b : blobs)
	{
		if( b.local_maxima_.size() == 0 )
		{
			b.local_maxima_.emplace_back( b.pos_.x, b.pos_.y, 0 );
		}
	}

	for (int i = 0, l = kps.size(); i < l ; ++i)
	{
		// each blob is drawn with its index inside blobs, as gray value inside contour_index_mask_
		// index 0 is a dummy since gray value 0 is the mask
		// find out inside which blob the keypoint falls by looking up the gray value under the key point coords
		// this enables us to skip the expensive point inside contour calculation
		const float blob_index = inside_blob( kps_ortho[ i ].pt.y, kps_ortho[ i ].pt.x );
		if ( blob_index > -1 )
		{
			blobs[ blob_index ].kps_ortho_.push_back( kps_ortho[ i ] );
			// also store kps in projected image space, in case we need them later 
			// for a bag of words search 
			blobs[ blob_index ].kps_.push_back( kps[ i ] );
		}
	}

	// first do a simple but strict pos and area match 
	vector<bool> is_matched( blobs.size(), false );
	for ( auto &p : persons_ )
	{
		float smallest_dist = numeric_limits<float>::max();
		float smallest_area_diff = smallest_dist;
		float smallest_combined = smallest_dist;
		int smallest_i = 0;

		for( int i = 0, s = blobs.size(); i < s; ++i )
		{
			if( !is_matched[ i ] )
			{
				auto vec = blobs[ i ].pos_ - p.blob_.pos_;
				
				float dist = myUtilities::fast_length( vec.x, vec.y );
				float area_diff = abs( blobs[ i ].area_ - p.blob_.area_ );
				float combined = dist + area_diff;
				if( combined < smallest_combined )
				{
					smallest_combined = combined;
					smallest_dist = dist;
					smallest_area_diff = area_diff;
					smallest_i = i;
				}
			}
		}
		if( smallest_dist < settings_.pos_match_radius_sq && smallest_area_diff < settings_.area_match_max_diff )
		{
			is_matched[ smallest_i ] = true;

			// find the nearest local maxima in blob 
			blob& b = blobs[ smallest_i ];

			float smallest_dist2 = numeric_limits<float>::max();
			int nearest_i = 0;
			for( int i = 0, l = b.local_maxima_.size(); i < l; ++i )
			{
				auto vec = b.local_maxima_[ i ].p - p.blob_.local_maxima_[ p.head_index_ ].p;

				float dist = myUtilities::fast_length( vec.x, vec.y );
				if( dist < smallest_dist2  )
				{
					smallest_dist2 = dist;
					nearest_i = i;
				}
			}
			p.new_match( move( b ), nearest_i );
		}
	}
	vector<blob> unmatched_blobs;
	unmatched_blobs.reserve( blobs.size() );

	for (int i = 0, s = is_matched.size(); i < s ; ++i)
	{
		if ( !is_matched[i] )
		{
			unmatched_blobs.push_back( move(blobs[ i ]) );
		}
	}


	// go through all blobs and match them against the active persons
	// map key is the index of the blob, value is a vector of person indices that matched to the blob
	blobs_matches blobs_to_persons = match( unmatched_blobs, persons_, settings_.maxima_match_radius_sq );
	

	// go through the matches of the new blobs with the active persons
	// and handle the blobs according to the number of matches
	// during loop persons_ can grow but existing indices dont change
	// disappeared_ can shrink 
	int vecsize = unmatched_blobs.size();
	for( int i = 0, l = blobs_to_persons.size(); i < l; ++i )
	{
		switch ( blobs_to_persons[ i ].size() )
		{
			case 0: {
				// no match in current persons so search in disappeared
				matches blob_to_disappeared = match( unmatched_blobs[ i ], disappeared_, settings_.disappeared_match_radius_sq );
				switch ( blob_to_disappeared.size() )
				{
					// NEW
					case 0:{
						if( unmatched_blobs[ i ].area_ > settings_.new_person_min_area )
						{
							persons_.emplace_back( move( unmatched_blobs[ i ] ) );
							cout << disappeared_.size() << " p in disappeared, new p id: "
								<< persons_.back().id_ << endl;
						}
						
						break;
					}
					// REAPPEARED
					case 1:{
						auto p = next( disappeared_.begin(), blob_to_disappeared[ 0 ].other_index );
						cout << "id " << p->id_ << " reappeared. ";
						p->new_match( move( unmatched_blobs[ i ] ), blob_to_disappeared[ 0 ].maxima.maxima_index );
						persons_.push_back( move( *p ) );
						disappeared_.erase( p );
						cout << disappeared_.size() << " persons in disappeared" << endl;
						break;
					}
					default:{
						// there should not be more than one match ?
						cout << "not handled: A new blob matched to more than one disappeared person" << endl;
						break;
					}
				}
				break;
			}
			// DIRECT MATCH
			case 1: {
				persons_[ blobs_to_persons[ i ][ 0 ].other_index ].new_match( move( unmatched_blobs[ i ] ),
						  blobs_to_persons[ i ][ 0 ].maxima.maxima_index );
				break;
			}
			// SEVERAL MATCHES
			default:{
				blob& b = unmatched_blobs[ i ];
				matches& ms = blobs_to_persons[ i ];

				Mat depth_masked = depth_ortho_.mul( b.mask_ );
				Mat refined_mask = Mat::zeros( contour_index_mask_.rows, contour_index_mask_.cols, CV_8UC1 );
				threshold( depth_masked, refined_mask, 1, 1, THRESH_BINARY );

				// use the matched local maxima as a starting label for watershed separation
				Mat lables = Mat::ones( contour_index_mask_.rows, contour_index_mask_.cols, CV_8U );
				lables -= refined_mask;
				int lablenum = 2;

				Mat b_descr = feature_detector_.describe( rgb_in, b.kps_ );
				vector<vector<KeyPoint>> split_off_blobs_kps;
				vector<vector<KeyPoint>> split_off_blobs_kps_ortho;
				split_off_blobs_kps.reserve( ms.size() );
				split_off_blobs_kps_ortho.reserve( ms.size() );
				
				
				Mat blob_label = Mat( b.mask_.rows, b.mask_.cols, CV_8UC1, Scalar( 0 ) );

				for (auto &m : ms)
				{
					if( b.kps_.size() > 0 && persons_[ m.other_index ].blob_.kps_.size() > 0 )
					{
						// generate a vector of all key points that belong to this match
						vector<Point> kps;
						kps.reserve(b.kps_.size());
	
						Mat p_descr = feature_detector_.describe( rgb_old_, persons_[ m.other_index ].blob_.kps_ );
						vector<DMatch> kp_matches = feature_detector_.match( p_descr, b_descr,
							persons_[ m.other_index ].blob_.kps_, b.kps_ );
	
						if ( kp_matches.size() > 0 )
						{
							vector<KeyPoint> split_off_kps;
							vector<KeyPoint> split_off_kps_ortho;
							split_off_kps.reserve( kp_matches.size() );
							split_off_kps_ortho.reserve( kp_matches.size() );
		
							for (auto &kp_match : kp_matches )
							{
								kps.push_back( b.kps_ortho_[ kp_match.queryIdx ].pt );
								split_off_kps.push_back( b.kps_[ kp_match.queryIdx ] );
								split_off_kps_ortho.push_back( b.kps_ortho_[ kp_match.queryIdx ] );
							}	
							
							split_off_blobs_kps.push_back( move( split_off_kps ) );
							split_off_blobs_kps_ortho.push_back( move( split_off_kps_ortho ) );
		
							// create convex Hull enclosing all keypoints
							vector<vector<Point>> hull_vec( 1 );
							hull_vec[ 0 ].reserve( kps.size() );
							convexHull( kps, hull_vec[ 0 ] );
		
							// draw the hull into an image so it can be used as a mask
 							drawContours( blob_label, hull_vec, 0, Scalar( lablenum ), FILLED );
						}
						else 
						{
							split_off_blobs_kps.push_back( vector<KeyPoint> {} );
							split_off_blobs_kps_ortho.push_back( vector<KeyPoint> {} );
						}
					}
					else
					{
						split_off_blobs_kps.push_back( vector < KeyPoint > {} );
						split_off_blobs_kps_ortho.push_back( vector < KeyPoint > {} );
					}
					++lablenum;
				}

				// draw heads last since keypoints are unreliable (because of watershed inaccuracy edge
				// key points can be assigned to the worng person and this error can scale up from frame to frame)
				// and head area should not be overdrawn by wrong key point hull
				lablenum = 2;
				for( auto &m : ms )
				{
					// TODO cache mat circle mat
					cv::circle( blob_label, b.local_maxima_[ m.maxima.maxima_index ].p, 
						maxima_.settings_.maximum_search_radius * 0.66f , Scalar( lablenum ), -1 );
					++lablenum;
				}

				// the hull can overlap the contour. only use hull pixels that are inside the blob mask (mul with refined mask)
				lables += blob_label.mul( refined_mask );

				Mat depth_mix = Mat::zeros( contour_index_mask_.rows, contour_index_mask_.cols, CV_8UC3 );

				const int from_to[] = { 0, 0 };
				mixChannels( &depth_masked, 1, &depth_mix, 1, from_to, 1 );

				Mat lables2;
				lables.convertTo( lables2, CV_32S );

				watershed( depth_mix, lables2 );
				Mat result;

				// -1 to remove the background label 
				lables2.convertTo( result, CV_8UC1, 1, -1 );


				// find the new contours
				vector<contour> new_contours_vec;

				// sometimes there are no black gaps between the result lables in the wsh image
				// finding an external contour does not work then. We have to create mask 
				// for every label and run the contourfinder on each

				auto it_m = ms.begin();
				for( int i = 1, s = ms.size(); i <= s; ++i )
				{
					Mat single_label = result == i;
					vector<contour> cs;
					findContours( single_label, cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

					
					if( cs.size() == 0 )
					{
						// Since using head marker in label this should not happen, but we keep it just in case
						cout << "deleted blob match to person id " << persons_[ it_m->other_index ].id_ 
							 << " after no contour was found from watershed result" << endl;
						it_m = ms.erase( it_m );

					}
					else
					{
						if( cs.size() > 1 )
						{
							// sometimes when blobs get cut off by noise and 
							// retouch, edge cases can happen, where there are more contours 
							// found (e.g. kp belongs to old cut off blob and is surrounded by kps of original blob)
							// watershed then can cut off a part of the blob and two contours emerge.
							// sort the vector largest first, smallest last and use only larges
							sort( cs.begin(), cs.end(),
								[]( contour &a, contour &b ){
								return a.size() > b.size();
							}
							);
						}

						new_contours_vec.push_back( move( cs[ 0 ] ) );
						++it_m;
					}

				}

				for ( int i = 0, end = ms.size(); i < end; ++i )
				{
					persons_[ ms[ i ].other_index ].new_match( blob{ move( new_contours_vec[ i ] ),
								   b.mask_.cols, b.mask_.rows,
								   move( split_off_blobs_kps[ i ] ),
								   move( split_off_blobs_kps_ortho[ i ] ),
								   b.local_maxima_[ ms[ i ].maxima.maxima_index ] 
								 }, 0 );

				}
				break;
			}
		}
	}


	// look for person objects that were not matched and mark them disappeared
	// TODO perhaps do this while we wait for the GPU result
	for( auto p_it = persons_.begin(); p_it != persons_.end(); )
	{
		if ( !p_it->matched_ )
		{
			p_it->disappeared_time_ = current_time;
			cout << "id " << p_it->id_ << " disappeared" << endl;
			disappeared_.push_back( move( *p_it ) );
			p_it = persons_.erase( p_it );
		} 
		else
			++p_it;
	}
	
#ifdef PC_PERF_MARK
	diagnostic::span *span_hands = new diagnostic::span( *marker_series1_, L"Find Hands and Gestures" );
#endif


	vector<person_result> result;
	result.reserve( persons_.size() );
	// TODO perhaps move this into person.new_match and person::make_new
	// or do this in parallel
	for ( auto &p : persons_ )
	{
		hand_finder_.find( p, depth_ortho_ );
		p.update_gestures();
		result.emplace_back( p );
	}
#ifdef PC_PERF_MARK
	delete span_hands;
#endif
	

#ifdef HT_SHOW_DEBUG_IMG
	//imshow("Contour Index Mask", test);
#endif // HT_SHOW_DEBUG_IMG
	rgb_old_ = rgb_in.clone();

	


	return result;
}

inline 
topdown_hand_tracker::maxima_match topdown_hand_tracker::match( const pacher::blob& b, 
	const pacher::person& p, int match_radius )
{
	// check if blob is similar to person
	// or in our simple case a local maxima has to be nearby

	float smallest_distance = numeric_limits<float>::max();
	int index_smallest = -1;
	for (int i = 0, s = b.local_maxima_.size(); i < s ; ++i)
	{
		const auto dist = b.local_maxima_[i].p - p.blob_.local_maxima_[p.head_index_].p;
		const float len = dist.x * dist.x + dist.y * dist.y;
		if( len < smallest_distance )
		{
			smallest_distance = len;
			if( len < match_radius )
			{
				index_smallest = i;
			}
		}
	} 
	return{ index_smallest, smallest_distance };
}

inline
topdown_hand_tracker::matches topdown_hand_tracker::match( const pacher::blob &blob, 
	const std::vector<pacher::person> &persons, 
	int match_radius )
{
	matches ms;
	for ( int j = 0, lp = persons.size(); j < lp; ++j )
	{
		if (!persons[j].matched_ )
		{
			maxima_match m_match = match( blob, persons[ j ], match_radius );
			if ( m_match.maxima_index > -1 )
			{
				ms.emplace_back( j, m_match );
			}
		}
	}

	// closest matches first
	sort( ms.begin(), ms.end(),
		[]( match_result &a, match_result &b ){
		return a.maxima.distance < b.maxima.distance;
	});

	// there should not be more matches than there are local maxima 
	if ( ms.size() > blob.local_maxima_.size() )
	{
		ms.erase( ms.begin() + blob.local_maxima_.size(), ms.end() );
	}
	return ms;
}

inline
topdown_hand_tracker::blobs_matches topdown_hand_tracker::match( const std::vector<pacher::blob> &blobs,
	const std::vector<pacher::person> &persons,
	int match_radius )
{
	blobs_matches bs_ms;
	// TODO try this in parallel with concurrent map 
	for ( int i = 0, end = blobs.size(); i < end; ++i )
	{
		bs_ms[ i ] = match( blobs[ i ], persons, match_radius );
	}
	return bs_ms;
}



topdown_hand_tracker::topdown_hand_tracker(topdown_hand_tracker::settings &s)
	:
	maxima_( s.maxima ), 
	back_sub_( s.background_sub ),
	reprojector_( s.reprojector ),
	blob_detector_( s.blob_detec ),
	feature_detector_( s.feature_detec ),
	hand_finder_( s.hand_finder ),
	settings_(s), 
	depth_buffer_size_( s.depth_elem_size * settings_.depthW * settings_.depthH ),
	rgb_buffer_size_( s.rgb_elem_size * settings_.rgbW * settings_.rgbH )
{

#ifdef PC_PERF_MARK
	marker_series_ = make_shared<marker_series>( L"Tracker" );
	marker_series1_ = make_shared<marker_series>( L"Tracker Sub steps" );
#endif

	depth_ortho_.create(s.depthH, s.depthW, CV_8UC1);
	color_ortho_.create( s.rgbH, s.rgbW, CV_8UC3 );

	color_ortho_masked_.create( s.rgbH, s.rgbW, CV_8UC3 );

	depth_backremoved_.create(s.depthH, s.depthW, CV_16UC1);
	color_backremoved_.create( s.rgbH, s.rgbW, CV_8UC3 );

	contour_index_mask_.create(s.depthH, s.depthW, CV_8UC1);
	
	using namespace ci::gl;
	rgb_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, rgb_buffer_size_, NULL, GL_STREAM_DRAW );

	depth_backrem_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, depth_buffer_size_, NULL, GL_STREAM_DRAW );

	

	auto colorForm = Texture::Format();
	colorForm.setInternalFormat( GL_RGB8 );
	colorForm.setPixelDataFormat( GL_BGR );
	colorForm.setPixelDataType( GL_UNSIGNED_BYTE );
	colorForm.enableMipmapping( true );
	colorForm.setMagFilter( GL_LINEAR );
	colorForm.setMinFilter( GL_LINEAR_MIPMAP_LINEAR );
	rgb_tex_ = Texture::create( s.rgbW, s.rgbH, colorForm );

	auto depthForm = Texture::Format();
	depthForm.setInternalFormat( GL_R16UI );
	depthForm.setPixelDataFormat( GL_RED_INTEGER );
	depthForm.setPixelDataType( GL_UNSIGNED_SHORT );
	depthForm.enableMipmapping( false );
	depthForm.setMagFilter( GL_NEAREST );
	depthForm.setMinFilter( GL_NEAREST );
	depth_backrem_tex_ = Texture::create( s.depthW, s.depthH, depthForm );



	
#ifdef HT_SHOW_DEBUG_IMG
	namedWindow("Contour Index Mask", WINDOW_AUTOSIZE);
#endif // HT_SHOW_DEBUG_IMG
}

cv::Mat topdown_hand_tracker::draw_persons_info( bool ortho )
{
#ifdef PC_PERF_MARK
	diagnostic::span span( *marker_series1_, L"Draw Info" );
#endif
	lock_guard<mutex> lock( mutex_ );
	Scalar red(0, 0, 255);
	Scalar white(255, 255, 255);
	Scalar green(0, 255, 0);
	Scalar orange(0, 127, 255);

	//Mat color_info;
	if( ortho )
	{
		color_info = color_ortho_.clone();
		int index = 0;
		for( const auto &p : persons_ )
		{

			// bounding box
			//rectangle(color_info_, p.blob_.bounding_b_, red);


			drawContours( color_info, p.blob_.contours_, 0, p.color_ );

			// head with hand finding radius

			//circle( color_info, p.blob_.local_maxima_[ p.head_index_ ].p,
			//	p.blob_.local_maxima_[ p.head_index_ ].v * hand_finder_.s_.height_to_headradius,
			//	green );
			//circle( color_info, p.blob_.local_maxima_[ p.head_index_ ].p,
			//	p.blob_.local_maxima_[ p.head_index_ ].v * hand_finder_.s_.height_to_handthreshold,
			//	green );
			circle( color_info, p.blob_.local_maxima_[ p.head_index_ ].p, 5, green );

			//// pos
			//circle( color_info, p.blob_.pos_, 5, orange );
			
			// hands
			//for (int i = 0; i < 2 ; ++i)
			//{
			//	if( p.hands_.h_[ i ].extended  )
			//	{
			//		circle( color_info, p.hands_.h_[ i ].pos.p, 7, green );
			//	}
			//	else if ( p.hands_.h_[ i ].pos.v != -1 )
			//	{
			//		circle( color_info, p.hands_.h_[ i ].pos.p, 7, red );
			//	}
			//}
			
			//// id 
			//putText( color_info, to_string( p.id_ ), p.blob_.bounding_b_.br(), FONT_HERSHEY_SIMPLEX, 0.5,
			//	p.hands_.touching_ ? green : red );
			//// age
			//shoot_gesture* sg = (shoot_gesture*)p.gestures_[ 0 ].get();

			//time_duration age = ptime( microsec_clock::universal_time() ) - p.creation_time_;
			//putText( color_info, to_string( age.minutes() ) + "." + to_string( age.seconds() ), 
			//	p.blob_.bounding_b_.br() + Point( 0, 20 ), FONT_HERSHEY_SIMPLEX, 0.5, 
			//	sg->hands_were_straight() ? green : red );
			//
			//// idx 
			//putText( color_info, "idx " + to_string( index ), p.blob_.bounding_b_.br() + Point( 0, 40 ), 
			//	FONT_HERSHEY_SIMPLEX, 0.5, 
			//	p.gestures_[ 0 ]->found() ? green : red );

			// key points
			for( const auto &kp : p.blob_.kps_ortho_ ) 
			{
				circle( color_info, kp.pt, 1, p.color_ );

			}
			++index;
		}

		// TODO use circular buffer to save history of blobs to be able to draw lines between matched kps


		// lines of matched key points
		//for( const auto &e : feature_detector_.good_matches_ )
		//{
		//	line( color_info, kp_new_ortho_[ e.queryIdx ].pt, kp_old_ortho_[ e.trainIdx ].pt, red );
		//}


	}
	else
	{
		color_info = color_backremoved_.clone();

		for( const auto &p : persons_ )
		{
			// id 
			//putText( color_info, to_string( p.id_ ), p.blob_.bounding_b_.br(), FONT_HERSHEY_SIMPLEX, 0.5, red );
			// age
			time_duration age = ptime( microsec_clock::universal_time() ) - p.creation_time_;
			//putText( color_info, to_string( age.minutes() ) + "." + to_string( age.seconds() ), 
				//p.blob_.bounding_b_.br() + Point( 0, 20 ), FONT_HERSHEY_SIMPLEX, 0.5, red );

			// draw key points
			for( const auto &kp : p.blob_.kps_ )
			{
				circle( color_info, kp.pt, 1, p.color_ );

			}
		}

		// lines of matched key points
		//for ( const auto &e : feature_detector_.good_matches_ )
		//{
		//	line( color_info, feature_detector_.kp_new_[ e.queryIdx ].pt,
		//		feature_detector_.kp_old_[ e.trainIdx ].pt, red );
		//}

	}

	return color_info;
}

cv::Mat topdown_hand_tracker::draw_persons_info_depth( bool ortho )
{
#ifdef PC_PERF_MARK
	diagnostic::span span( *marker_series1_, L"Draw Info" );
#endif
	lock_guard<mutex> lock( mutex_ );
	Scalar red( 0, 0, 255 );
	Scalar white( 255, 255, 255 );
	Scalar green( 0, 255, 0 );
	Scalar orange( 0, 127, 255 );

	Mat depth_info;
	if ( ortho )
	{
		depth_info = depth_ortho_.clone();
		int index = 0;
		for ( const auto &p : persons_ )
		{

			for ( local_maxima::PointValue pv : p.blob_.local_maxima_ )
			{
				circle( depth_info, pv.p, maxima_.settings_.maximum_search_radius, Scalar(255) );
			}

			++index;
		}

	}
	else
	{


	}

	return depth_info;
}

inline
void topdown_hand_tracker::upload_rgb( cv::Mat &rgb_in )
{
#ifdef PC_PERF_MARK
	diagnostic::span span( *marker_series1_, L"Buffer RGB" );
#endif

	// copy rgb pixels into PBO
	rgb_pbo_->bufferSubData( 0, rgb_buffer_size_, rgb_in.data );
	// update texture with pbo async
	rgb_tex_->update( rgb_pbo_, GL_BGR, GL_UNSIGNED_BYTE );
}

inline
void topdown_hand_tracker::upload_depth()
{
#ifdef PC_PERF_MARK
	diagnostic::span span( *marker_series1_, L"Buffer Depth" );
#endif
	// copy depth pixels into PBO
	depth_backrem_pbo_->bufferSubData( 0, depth_buffer_size_, depth_backremoved_.data );
	// update texture with pbo async  
	depth_backrem_tex_->update( depth_backrem_pbo_, GL_RED_INTEGER, GL_UNSIGNED_SHORT );
}

inline
int topdown_hand_tracker::inside_blob( float y, float x ){
	return contour_index_mask_.at<uchar>( roundf( y ), roundf( x )  ) - 1;
}

void topdown_hand_tracker::reset()
{
	person::reset_next_id();
	persons_.clear();
	disappeared_.clear();
}



