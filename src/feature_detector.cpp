/*!
 * @file	feature_detector.cpp
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

#include "feature_detector.h"

#include <opencv2/highgui/highgui.hpp>

#include <ppl.h>
#include <concurrent_vector.h>

#include <string>
#include <limits>


//#define FD_SHOW_DEBUG_IMG

using namespace cv;
using namespace std;
#ifdef PC_PERF_MARK
using namespace concurrency;
using namespace concurrency::diagnostic;
#endif // PC_PERF_MARK

feature_detector::feature_detector(const feature_detector::settings &s)
	:
	settings_(s),
	cv_detector_( FastFeatureDetector::create( settings_.detector_threshold ) )
{	
#ifdef PC_PERF_MARK
	marker_series_ = make_shared<marker_series>( L"Feature Processing" );
#endif
#ifdef FD_SHOW_DEBUG_IMG
	//namedWindow( "FeatureDetector", WINDOW_AUTOSIZE );
	namedWindow( "FeatureDetector2", WINDOW_AUTOSIZE );
#endif // FD_SHOW_DEBUG_IMG
}


std::vector<cv::KeyPoint> feature_detector::detect( const cv::Mat &rgb_in, const cv::Mat &mask /*= cv::Mat()*/ )
{
#ifdef PC_PERF_MARK	
	span span( *marker_series_, L"detect Key Points" );
#endif

	//equalizeHist( gray, gray );


	Mat mask_erode;
	if ( settings_.mask_erode_interations > 0 )
	{
		mask_erode = mask.clone();
		dilate( mask_erode, mask_erode, erode_kernel_,
			Point( -1, -1 ), settings_.mask_erode_interations );
	}
	else
		mask_erode = mask;

	vector<cv::KeyPoint> kps;
#ifdef PC_PERF_MARK	
	marker_series_->write_message( L"Detect" );
#endif
	cv_detector_->detect( rgb_in, kps, mask_erode );

#ifdef FD_SHOW_DEBUG_IMG
	Mat outImg2;
	drawKeypoints( rgb_in, kps, outImg2, Scalar::all( -1 ), DrawMatchesFlags::DEFAULT );
	imshow( "FeatureDetector2", outImg2 );
#endif

	return kps;
}


cv::Mat feature_detector::describe( const cv::Mat &rgb_in, std::vector<cv::KeyPoint> &kp )
{
#ifdef PC_PERF_MARK	
	span span( *marker_series_, L"describe Key Points" );
#endif
	Mat descriptor;
	cv_extractor_->compute( rgb_in, kp, descriptor );
	return descriptor;
}



std::vector<cv::DMatch> feature_detector::match( const cv::Mat &descr_old, const cv::Mat &descr_new,
	const std::vector<cv::KeyPoint> &kp_old, const std::vector<cv::KeyPoint> &kp_new )
{
#ifdef PC_PERF_MARK	
	span span( *marker_series_, L"match Key Points" );
#endif
	vector<vector<cv::DMatch>> matches;
	double max_dist = 0; double min_dist = 100;
	
	if ( descr_new.rows > 0 && descr_old.rows > 0  )
	{
		// match only if there is something to match
		cv_matcher_.knnMatch( descr_new, descr_old, matches, 2 );
	}
	
	/*	for_each( matches.begin(), matches.end(), [ &]( vector<DMatch> &e )
		{
		double dist = e[ 0 ].distance;
		if ( dist < min_dist ) min_dist = dist;
		if ( dist > max_dist ) max_dist = dist;
		} );
		const float max_descr_distance = min_dist = max( min_dist * settings_.max_descriptor_distance_multi, 0.02 );
		*/
		
	//concurrent_vector<cv::DMatch> good_matches;
	vector<cv::DMatch> good_matches;
	good_matches.reserve( matches.size() );

	//	Filter matches according to descriptor distance and euclidean distance in image space
	//parallel_for_each( matches.begin(), matches.end(), [ &]( vector<DMatch> &e )
	for_each( matches.begin(), matches.end(), [ &]( vector<DMatch> &e )
	{
		using namespace cv;

		if ( e.size() > 1 && 
				e[ 0 ].distance < e[ 1 ].distance * settings_.match_1_2_ratio &&
				e[ 0 ].distance < settings_.max_descriptor_distance_multi )
		{
			const Point2f delta = kp_new[ e[ 0 ].queryIdx ].pt - kp_old[ e[ 0 ].trainIdx ].pt;

			const float img_distance = delta.x * delta.x + delta.y * delta.y;

			if ( img_distance < settings_.max_keypoint_distance )
			{
				good_matches.push_back( e[ 0 ] );
			}
		}
	} );

	//return{ good_matches.begin(), good_matches.end() };
	return good_matches;


//	Mat outImg;
//	vector<DMatch> gm;
//	
//	if ( good_matches_.size() > 0 )
//	{
//		gm.insert( gm.begin(), good_matches_.begin(), good_matches_.end() );
//		drawMatches( rgb_in, kp_new_, img_old_, kp_old_, gm, outImg, Scalar::all( -1 ), Scalar::all( -1 ), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//		imshow( "FeatureDetector", outImg );
//	}

//	
//

//	if ( kps_max_ < kp_new_.size() ) kps_max_ = kp_new_.size();
//	if ( kps_min_ > kp_new_.size() && kp_new_.size() > 0 ) kps_min_ = kp_new_.size();
//	if ( matched_max_ < good_matches_.size() ) matched_max_ = good_matches_.size();
//	if ( matched_min_ > good_matches_.size() && good_matches_.size() > 0 ) matched_min_ = good_matches_.size();
//
//	float speed = 0.999;
//	average_kps_ = average_kps_ * speed + kp_new_.size() * ( 1 - speed );
//	average_matched_ = average_matched_ * speed + good_matches_.size() * ( 1 - speed );
//	putText( outImg2, "Kpts avg: " + to_string( average_kps_ ) +
//					  " min: " + to_string( kps_min_ ) + " max: " + to_string( kps_max_ ), 
//		Point( 10, 20 ), FONT_HERSHEY_SIMPLEX, 0.5, Scalar( 255, 255, 255 ) );
//	putText( outImg2, "Matched avg: " + to_string( average_matched_ ) +
//					  " min: " + to_string( matched_min_ ) + " max: " + to_string( matched_max_ ), 
//		Point( 10, 40 ), FONT_HERSHEY_SIMPLEX, 0.5, Scalar( 255, 255, 255 ) );
//	putText( outImg2, "Min Descr Dist: " + to_string( min_dist ) +
//		" Max Descr Dist: " + to_string( max_dist ), 
//		Point( 10, 60 ), FONT_HERSHEY_SIMPLEX, 0.5, Scalar( 255, 255, 255 ) );

//
//	img_old_ = rgb_in.clone();
//#endif
}


void feature_detector::detector_threshold( int threshold )
{
	cv_detector_->setThreshold( threshold );
}
