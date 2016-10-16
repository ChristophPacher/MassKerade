/*!
 * @file	local_maxima.cpp
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

#include "local_maxima.h"
#include "blob.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include <boost/geometry.hpp>


#include <ppl.h>
#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK
#include "boost_geometry_opencv_register.h"

//#define LM_SHOW_DEBUG_IMG

using namespace cv;
using namespace std;
using namespace concurrency;
namespace bg = boost::geometry;



std::mutex local_maxima::mutex_{};

local_maxima::local_maxima(const settings &s) 
	:
	settings_(s)
{
	set_search_radius( settings_.maximum_search_radius );

#ifdef LM_SHOW_DEBUG_IMG
	namedWindow("Local Maxima", WINDOW_AUTOSIZE);
#endif // LM_SHOW_DEBUG_IMG
}

std::vector<local_maxima::PointValue> local_maxima::detect( const cv::Mat &depth, const cv::Mat &mask )
{
	lock_guard<mutex> lock( mutex_ );

	assert( depth.type() == CV_8UC1 && mask.type() == CV_8UC1 );

	Mat depth_scaled, depth_bordered, depth_scaled_bordered, 
		mask_scaled, mask_scaled_bordered, map;


#ifdef PC_PERF_MARK
	diagnostic::marker_series marker_series( L"Maxima" );
	{
		diagnostic::span span( marker_series, L"maxima setup" );
#endif // PC_PERF_MARK

		if ( settings_.image_scale == 1.f )
		{
			copyMakeBorder( depth, depth_scaled_bordered, k_scaled_half_, k_scaled_half_, k_scaled_half_, k_scaled_half_, BORDER_CONSTANT, Scalar( 0 ) );
			copyMakeBorder( mask, mask_scaled_bordered, k_scaled_half_, k_scaled_half_, k_scaled_half_, k_scaled_half_, BORDER_CONSTANT, Scalar( 0 ) );
		} 
		else
		{
			resize( depth, depth_scaled, Size( 0, 0 ), settings_.image_scale, settings_.image_scale );
			resize( mask, mask_scaled, Size( 0, 0 ), settings_.image_scale, settings_.image_scale );

			copyMakeBorder( depth_scaled, depth_scaled_bordered, k_scaled_half_, k_scaled_half_, k_scaled_half_, k_scaled_half_, BORDER_CONSTANT, Scalar( 0 ) );
			copyMakeBorder( mask_scaled, mask_scaled_bordered, k_scaled_half_, k_scaled_half_, k_scaled_half_, k_scaled_half_, BORDER_CONSTANT, Scalar( 0 ) );

			
		}
		
		copyMakeBorder( depth, depth_bordered, k_half_, k_half_, k_half_, k_half_, BORDER_CONSTANT, Scalar( 0 ) );

		map = Mat( depth_scaled_bordered.rows, depth_scaled_bordered.cols, CV_8UC1, Scalar( 0 ) );

		//bilateralFilter(depth_temp, depth_c, 5, 20, 20);

#ifdef PC_PERF_MARK
	}
#endif // PC_PERF_MARK



#ifdef LM_SHOW_DEBUG_IMG
	//Mat debugout = depth.clone();
	debugout = depth.clone();
#endif // LM_SHOW_DEBUG_IMG



#ifdef PC_PERF_MARK
	diagnostic::span span2( marker_series, L"maxima loop" );
#endif // PC_PERF_MARK

	maxima_cncrr.clear();
	maxima_cncrr.reserve( 20 );

	parallel_for( k_scaled_half_, depth_scaled_bordered.rows - k_scaled_half_, [ & ]( int y ){
		for( int x = k_scaled_half_, endx = depth_scaled_bordered.cols - k_scaled_half_; x < endx; ++x )
		{
			int index = depth_scaled_bordered.cols * y + x;
			if ( mask_scaled_bordered.data[ index ] > 0 && map.data[ index ] == 0 )
			{
				int diff = is_maximum( index, depth_scaled_bordered, kernel_pixel_offsets_scaled_ );
				if( diff > settings_.min_diff )
				{
					PointValue val, val_centroid;
					// remove border and scale back 
					val.p.x = ( x - k_scaled_half_ ) / settings_.image_scale;
					val.p.y = ( y - k_scaled_half_ ) / settings_.image_scale;

					
					if ( val.p.x < 0 || val.p.y < 0  )
					{
						// TODO check why some few values are below zero 
						cout << "!!!!!!!!!!! local maxima coords below zero. x: " << x << " y: " << y << endl;
						assert( false );
					}
					else
					{
						val.v = depth_scaled_bordered.at<uchar>( y, x );

						// mark the area around the location to be not searched 
						// anymore for maxima
						for ( int i = 0, end = kernel_pixel_offsets_scaled_.size(); i < end; ++i )
						{
							map.data[ index + kernel_pixel_offsets_scaled_[ i ] ] = 255;
						}

						val_centroid = get_maximum_blob_centroid( val, depth_bordered, kernel_pixel_offsets_, k_half_ );

						if ( val_centroid.p.x < 0 || val_centroid.p.y < 0 )
						{
							// TODO check why some few values are below zero
							// hint: {{x=432 y=138 },{x=431 y=139 },{x=431 y=140 },{x=429 y=142 },{x=431 y=140 },{x=431 y=139 },{x=432 y=138 }}
							// centroid returned 0/0 for this contour. investigate further
							cout << "!!!!!!!!!!! local maxima coords below zero. x: " << x << " y: " << y << endl;
							assert( false );
						}
						else
						{
							maxima_cncrr.push_back( val_centroid );
						}
					}
				}
			}
		}
	});

	vector<PointValue> filtered;
	filtered.reserve( maxima_cncrr.size() );

	// because of multithreading sometimes two maximas with the 
	// same height, that are too close to each other are found, filter these out
	int thresh = settings_.maximum_search_radius * settings_.maximum_search_radius;
	vector<bool> keep( maxima_cncrr.size(), true );


	for( int i = 0, s = maxima_cncrr.size(); i < s; ++i )
	{
		for( int n = i + 1; n < s; ++n )
		{
			const auto vec = maxima_cncrr[ n ].p - maxima_cncrr[ i ].p;
			const auto len = vec.x * vec.x + vec.y * vec.y;
			if ( len < thresh   )
			{
				if( maxima_cncrr[ i ].v < maxima_cncrr[ n ].v )
				{
					keep[ i ] = false;
				}
				else
				{
					keep[ n ] = false;
#ifdef LM_SHOW_DEBUG_IMG
					circle( debugout, maxima_cncrr[ n ].p, settings_.maximum_search_radius, Scalar( 125 ) );
#endif // LM_SHOW_DEBUG_IMG
				}
			}
		}
		if( keep[ i ] )
		{
			filtered.push_back( maxima_cncrr[ i ] );
#ifdef LM_SHOW_DEBUG_IMG
			circle( debugout, maxima_cncrr[ i ].p, settings_.maximum_search_radius, Scalar( 255 ) );
		} 
		else
		{
			circle( debugout, maxima_cncrr[ i ].p, settings_.maximum_search_radius, Scalar( 125 ) );
#endif // LM_SHOW_DEBUG_IMG
		}
	}



#ifdef LM_SHOW_DEBUG_IMG
	imshow( "Local Maxima", debugout );
#endif // LM_SHOW_DEBUG_IMG

	return filtered;
}

void local_maxima::set_search_radius( int maximum_search_radius )
{
	lock_guard<mutex> lock( mutex_ );

	settings_.maximum_search_radius = maximum_search_radius;

	const int r_double = settings_.maximum_search_radius * 2;

	init_kernel( kernel_pixel_offsets_scaled_, k_scaled_half_, 
		r_double * settings_.image_scale, settings_.image_scale);

	init_kernel( kernel_pixel_offsets_, k_half_, r_double, 1.f);
}


void local_maxima::set_image_scale( float scale )
{
	settings_.image_scale = scale;
	set_search_radius( settings_.maximum_search_radius );
}

void local_maxima::init_kernel( std::vector<int> &kernel_offsets, int &kernel_half, int search_window, float scale )
{
	if( search_window % 2 == 0 ) ++search_window;

	Mat kernel = getStructuringElement( MORPH_ELLIPSE, Size( search_window, search_window ) );
	kernel_offsets.clear();
	kernel_offsets.reserve( kernel.rows * kernel.cols );
	kernel_half = ( kernel.rows - 1 ) / 2;

	auto k_ptr = kernel.data;

	const int masked_image_width = settings_.image_width * scale + 2 * kernel_half;
	for( int y = 0, endy = kernel.rows; y < endy; ++y )
	{
		const int offset = -kernel_half + ( y - kernel_half ) * masked_image_width;
		for( int x = 0, endx = kernel.cols; x < endx; ++x, ++k_ptr )
		{
			if( *k_ptr > 0 )
			{
				kernel_offsets.push_back( x + offset );
			}
		}
	}
}

int local_maxima::is_maximum( int index, cv::Mat depth_bordered, const std::vector<int>& kernel_offsets )
{
	const int c_pixel = depth_bordered.data[ index ];
	int diff = -1;
	int min_val = (numeric_limits<int>::max)();
	int max_val = 0;
	for ( int i = 0, end = kernel_offsets.size(); i < end; ++i )
	{
		const int v = depth_bordered.data[ index + kernel_offsets[ i ] ];
		if ( v < min_val && v != 0 )
		{
			min_val = v;
		}
		if ( v > max_val )
		{
			max_val = v;
		}
	}
	if ( max_val == c_pixel )
	{
		diff = max_val - min_val;
	}
	return diff;
}

local_maxima::PointValue local_maxima::search_maximum( int x, int y, cv::Mat depth_bordered, 
	const std::vector<int>& kernel_offsets, int border )
{
	const int index = x + border + ( (y + border) * depth_bordered.cols );

	PointValue val{ 0.f, 0.f, -1};
	
	int index_max = index;
	for ( int i = 0, end = kernel_offsets.size(); i < end; ++i )
	{
		const int index_current = index + kernel_offsets[ i ];
		const int v = depth_bordered.data[ index_current ];
		if ( v > val.v )
		{
			val.v = v;
			index_max = index_current;
		}
	}
	val.p.y = int(index_max / depth_bordered.cols);
	val.p.x = index_max - ( val.p.y * depth_bordered.cols ) - border;
	val.p.y -= border;
	return val;
}

local_maxima::PointValue local_maxima::get_maximum_blob_centroid( local_maxima::PointValue m, cv::Mat depth_bordered,
	const std::vector<int>& kernel_offsets, int border )
{
	const int index = m.p.x + border + ( ( m.p.y + border ) * depth_bordered.cols );

	Mat blob_img( depth_bordered.rows, depth_bordered.cols, depth_bordered.type(), Scalar( 0 ) );
	const int thresh = m.v - settings_.maximum_blob_range;
	for ( int i = 0, end = kernel_offsets.size(); i < end; ++i )
	{
		const int index_current = index + kernel_offsets[ i ];
		const int v = depth_bordered.data[ index_current ];
		if ( v >= thresh )
		{
			blob_img.data[ index_current ] = v;
		}
	}

	vector<vector<Point>> cs;
	findContours( blob_img, cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

	if ( cs.size() > 0 )
	{
		sort( cs.begin(), cs.end(),
			[]( vector<Point> &a, vector<Point> &b ){
			return a.size() > b.size();
		} );
		Point2f pos;
		bg::correct( cs[ 0 ] );
		bg::centroid( cs[ 0 ], pos );
		
		m.p.x = pos.x - border;
		m.p.y = pos.y - border;
	}
	return m;
}