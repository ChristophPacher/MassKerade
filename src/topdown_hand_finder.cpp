/*!
 * @file	topdown_hand_finder.cpp
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

#include "topdown_hand_finder.h"

#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

#include "blob.h"

#include "pacher_utilities.h"

using namespace cv;
using namespace std;
using namespace pacher;

//#define HF_SHOW_DEBUG_IMG

/*
#include <vector>
using std::vector;
#include <iostream>
using std::ostream;

template<typename T>
ostream& operator<< ( ostream& out, const vector<T>& v ) {
	out << "[";
	size_t last = v.size() - 1;
	for( size_t i = 0; i < v.size(); ++i ) {
		out << v[ i ];
		if( i != last )
			out << ", ";
	}
	out << "]";
	return out;
}
*/

// hands with depth value -1 were not found
void topdown_hand_finder::find( pacher::person &p, const cv::Mat &depth )
{
	assert( depth.type() == CV_8UC1 );

	Mat mask = depth.mul( p.blob_.mask_ );
	p.hands_.reset();



	// cut out a circle with a radius that represents the hands detection threshold
	// the blobs that remain represent the hands
	circle( mask, p.blob_.local_maxima_[ p.head_index_ ].p, p.blob_.local_maxima_[ p.head_index_ ].v * s_.height_to_headradius, Scalar( 0 ), -1 );

	#ifdef HF_SHOW_DEBUG_IMG
	Mat debugout = mask.clone();
	#endif // HF_SHOW_DEBUG_IMG
	
	vector<contour> cs;
	findContours( mask, cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
	
	// compute the area of the contours and only use the ones that are big enough
	vector<contour_info> cs_info;
	for ( auto &c : cs)
	{
		double a = contourArea( c );
		if (a > s_.hands_min_size )
		{
			cs_info.emplace_back( move( c ), a );
		}
	}

	/*
	vector<Point> c = { { 356, 339 }, { 355, 340 }, { 353, 340 }, { 350, 343 }, { 349, 343 }, { 347, 345 }, { 344, 345 }, { 343, 346 }, { 334, 346 }, { 330, 350 }, { 331, 350 }, { 332, 351 }, { 334, 351 }, { 335, 352 }, { 336, 352 }, { 338, 354 }, { 339, 354 }, { 340, 355 }, { 340, 354 }, { 341, 353 }, { 342, 353 }, { 343, 352 }, { 344, 352 }, { 345, 351 }, { 349, 351 }, { 350, 350 }, { 352, 350 }, { 353, 349 }, { 354, 349 }, { 355, 350 }, { 356, 350 }, { 357, 351 }, { 357, 353 }, { 354, 356 }, { 354, 357 }, { 352, 359 }, { 353, 360 }, { 350, 363 }, { 350, 364 }, { 351, 365 }, { 351, 366 }, { 353, 368 }, { 353, 369 }, { 355, 371 }, { 355, 372 }, { 356, 371 }, { 356, 369 }, { 358, 367 }, { 358, 366 }, { 361, 363 }, { 361, 357 }, { 362, 356 }, { 362, 354 }, { 364, 352 }, { 364, 351 }, { 367, 348 }, { 368, 348 }, { 369, 347 }, { 367, 345 }, { 367, 343 }, { 366, 343 }, { 363, 340 }, { 359, 340 }, { 358, 339 } };
	Mat img(480, 640, CV_8UC3, Scalar(0, 0, 0));
	Scalar colorline(50, 0, 0);


	
	vector<int> h;
	convexHull( c, h );
	vector<Point> hp;
	for( const auto &e : h )
	{
		hp.push_back( c[ e ] );
	}
	cv::polylines( img, hp, true, Scalar( 0, 255, 0 ), 1 );

	for( int i = 0, s = c.size(); i < s; ++i )
	{
		if( i == s - 1 )
		{
			line( img, c[ i ], c[ 0 ], colorline, 1 );
		}
		else
		{
			line( img, c[ i ], c[ i + 1 ], colorline, 1 );
		}

		colorline += Scalar( 2, 0, 0 );
	}

	vector<Vec4i> d;
	convexityDefects( c, h, d );

	for( const auto &e : d )
	{
		circle( img, c[ e[ 2 ] ], 1, Scalar( 0, 0, 255 ), 1 );
	}
	cout << "contour: " << endl;
	cout << c << endl;

	cout << "hull: " << endl;
	cout << h << endl;

	cout << "defects: " << endl;
	cout << d << endl;
	*/



	if( cs_info.size() == 1 )
	{
		// if there is only one contour found check for convexity defect that is deep enough to 
		// be accepted as the form of two touching extended hands
		vector<int> hull;
		convexHull( cs_info[ 0 ].c, hull, false );
		vector<Vec4i> defects;
		convexityDefects( cs_info[ 0 ].c, hull, defects );


		vector<Point> hullpoints;
		for (const auto &e : hull)
		{
			hullpoints.push_back( cs_info[ 0 ].c[ e ] );
		}
		Mat hull_img(depth.rows, depth.cols, depth.type(), Scalar(0) );

		cv::polylines( hull_img, hullpoints, true, Scalar( 255 ), 1 );

		vector<Vec4i> defects_filtered;

		for( const auto &e : defects )
		{			
			if( e[ 3 ] / 256.f > 10 && 
				// convexityDefects often has wrong results with deep defects that lie on the hull!!
				// we filter these out
				hull_img.at<uchar>( cs_info[ 0 ].c[ e[ 2 ] ].y, cs_info[ 0 ].c[ e[ 2 ] ].x ) != 255 )
			{
				defects_filtered.push_back( e );
			}

		}
		if (defects_filtered.size() > 1)
		{
			// if there are several defects we use the deepest
			sort( defects_filtered.begin(), defects_filtered.end(),
				[]( Vec4i &a, Vec4i &b ){
				return a[ 3 ] > b[ 3 ];
			} );
		}
		if( defects_filtered.size() > 0 && defects_filtered[0][3] > s_.defect_min_depth )
		{
			auto result = find_farthest( cs_info[ 0 ].c, p.blob_.local_maxima_[ p.head_index_ ].p );
			p.hands_.h_[ 0 ].pos.p = cs_info[ 0 ].c[ get<0>(result) ];

			//p.hands_.h_[ 0 ].pos.p = cs_info[ 0 ].c[ defects_filtered[ 0 ][ 2 ] ];
			p.hands_.h_[ 0 ].pos.v = depth.at<uchar>( p.hands_.h_[ 0 ].pos.p.y, p.hands_.h_[ 0 ].pos.p.x );
			p.hands_.h_[ 0 ].extended = true;
			p.hands_.h_[ 1 ] = p.hands_.h_[ 0 ];
			p.hands_.touching_ = true;
			p.hands_.angle_ = 0.f;

#ifdef HF_SHOW_DEBUG_IMG
			circle( debugout, cs_info[ 0 ].c[ defects_filtered[ 0 ][ 2 ] ], 1, Scalar( 255 ), 1 );
#endif // HF_SHOW_DEBUG_IMG
		}
	}
	// in case we did not find touching hands also search for one hand
	if(p.hands_.touching_ == false)
	{
		// if there are more contours found we define that the biggest two are the extended hands.
		// we find the contour points that are farthest away to the head.
		// this does not make sure that the left hand is at index 0 and the right hand at index 1

		// it would be really hard to find out where is left and right. I dont know a way to 
		// analyze the contour to find it out. one could use the move direction of the person
		// but its all wrong as soon one walks backwards. one could define that the walking direction
		// when entering the camera view is always front. 
		sort( cs_info.begin(), cs_info.end(),
			[]( contour_info &a, contour_info &b ){
			return a.area > b.area;
		} );

		for( int i = 0, s = cs_info.size(); i < 2 && i < s; ++i )
		{
			auto result = find_farthest( cs_info[ i ].c, p.blob_.local_maxima_[ p.head_index_ ].p );

			p.hands_.h_[ i ].pos.p = cs_info[ i ].c[ get<0>(result) ];
			p.hands_.h_[ i ].pos.v = depth.at<uchar>( p.hands_.h_[ i ].pos.p.y, p.hands_.h_[ i ].pos.p.x );
			float thresh = p.blob_.local_maxima_[ p.head_index_ ].v * s_.height_to_handthreshold;
			if ( get<1>( result ) > thresh * thresh )
			{
				p.hands_.h_[ i ].extended = true;
			}
		}
	}

	if ( p.hands_.h_[0].extended && p.hands_.h_[1].extended )
	{
		Vec2f v0 = p.hands_.h_[ 0 ].pos.p - p.blob_.local_maxima_[ p.head_index_ ].p;
		Vec2f v1 = p.hands_.h_[ 1 ].pos.p - p.blob_.local_maxima_[ p.head_index_ ].p;
		float l0 = norm( v0 );
		float l1 = norm( v1 );
		p.hands_.angle_ = 0.f;

		if ( l0 > 0.f && l1 > 0.f )
		{
			v0 /= l0;
			v1 /= l1;
			float dot = v0.dot( v1 );
			if ( dot <= 1 )
			{
				p.hands_.angle_ = acos(dot);
			}
		}
	}

#ifdef HF_SHOW_DEBUG_IMG

	imshow( "Hand Finder", debugout );
#endif
}

topdown_hand_finder::topdown_hand_finder( settings &s )
	:
	s_(s)
{
#ifdef HF_SHOW_DEBUG_IMG
	namedWindow( "Hand Finder", WINDOW_AUTOSIZE );
#endif // HF_SHOW_DEBUG_IMG
}

std::tuple<int, int> topdown_hand_finder::find_farthest( const pacher::contour &c, cv::Point from ) const
{
	int max_dist = 0;
	int farthest = 0;
	for ( int j = 0, s = c.size(); j < s; ++j )
	{
		Point vec = c[ j ] - from;
		int dist = abs( vec.x * vec.x + vec.y * vec.y );
		if ( dist > max_dist )
		{
			max_dist = dist;
			farthest = j;
		}
	}
	return make_tuple(farthest, max_dist);
}
