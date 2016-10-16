/*!
 * @file	background_substractor.cpp
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

#include "background_substractor.h"

#include <math.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h >
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>
#include <opencv2/core/utility.hpp>

#include <ppl.h>



#include "pacher_utilities.h"


//#define BK_SHOW_DEBUG_IMG

using namespace cv;
using namespace std;
#ifdef PC_PERF_MARK
using namespace concurrency;
using namespace concurrency::diagnostic;
#endif // PC_PERF_MARK

background_substractor::background_substractor( const background_substractor::settings &s ) :
s_(s),
backgr_average_( s.h, s.w, s.type, Scalar( 3600 ) ),
background_( s.h, s.w, s.type, Scalar( 3600 ) ),
background_timer_( s.h, s.w, s.type, Scalar( 0 ) )
{
	prev_images_.set_capacity( s_.num_prev_images );

#ifdef PC_PERF_MARK
	marker_series_ = make_shared<marker_series>( L"Background Sub" );
#endif
#ifdef BK_SHOW_DEBUG_IMG
	namedWindow("BKSubstrktrBack", WINDOW_AUTOSIZE);
	namedWindow("BKSubstrktrAvg", WINDOW_AUTOSIZE);
	namedWindow( "BKSubstrktrTimer", WINDOW_AUTOSIZE );
#endif // FD_SHOW_DEBUG_IMG
}

void background_substractor::process(cv::Mat &depth_in, cv::Mat &depth_out, cv::Mat &color_in, cv::Mat &color_out)
{
#ifdef PC_PERF_MARK
	span *hole_fill_span = new span( *marker_series_, 1,  L"hole_fill" );
#endif // PC_PERF_MARK
	Mat depth_in_copy = depth_in.clone();
	assert( depth_in_copy.type() == CV_16UC1 );

	Mat holes, holes_roi, depth_in_8, depth_in_erode, depth_in_erode_roi;
	depth_in_copy.copyTo( depth_in_erode );

	const double max_8 = ( numeric_limits<unsigned char>::max )();
	const double max_16 = ( numeric_limits<unsigned short>::max )();
	depth_in_copy.convertTo( depth_in_8, CV_8UC1, max_8 / max_16 );


	// find black holes
	//threshold( img_in_8, holes, 0.f, 255.f, THRESH_BINARY_INV );
	holes = ( depth_in_8 == 0 );

	holes_roi = holes( Rect( s_.roi_x, s_.roi_y, s_.roi_w, s_.roi_h ) );
	depth_in_erode_roi = depth_in_erode( Rect( s_.roi_x, s_.roi_y, s_.roi_w, s_.roi_h ) );

	// add highest 16bit unsigned int value to holes to make erode work as we expect it
	add( depth_in_erode_roi, Scalar( ( numeric_limits<unsigned short>::max )() ), depth_in_erode_roi, holes_roi );

	// erode (replace kernel center with smallest value under kernel window)
	// remember depth image values get higher the farther they are away
	// erode basically fills the hole with the smallest value it can 
	// find inside the kernel window
	erode( depth_in_erode_roi, depth_in_erode_roi,
		getStructuringElement( MORPH_ELLIPSE, Size( s_.erode_size, s_.erode_size ), Point( -1, -1 ) ),
			Point( -1, -1 ), s_.erode_iterations, BORDER_CONSTANT, Scalar(0) );

	// use eroded image to fill black holes in uneroded image
	add( Scalar( 0 ), depth_in_erode, depth_in_copy, holes );
#ifdef PC_PERF_MARK
	delete hole_fill_span;
#endif // PC_PERF_MAR

	// Mat depth_in_copy_float;
	// depth_in_copy.convertTo( depth_in_copy_float, CV_32FC1, 1.0 / max_16 );


	// TODO remove max 16bit values that survived the erode iterations before noise filter


#ifdef PC_PERF_MARK
	span *median_blur_span = new span( *marker_series_, 1,  L"medien_blur" );
#endif // PC_PERF_MARK
	for ( int i = 0; i < s_.median_iterations; ++i )
	{
		medianBlur( depth_in_copy, depth_in_copy, s_.median_size );
		// ximgproc::guidedFilter( depth_in_copy_float, depth_in_copy_float, depth_in_copy_float, s_.median_size, 0.2 );
	}
#ifdef PC_PERF_MARK
	delete median_blur_span;
#endif // PC_PERF_MAR

	// depth_in_copy_float.convertTo( depth_in_copy, CV_16UC1, max_16 );


	
	// TODO try fastNlMeansDenoising and inpaint() but needs U8/32F

	// TODO combine morph hole filling and average background model 
	// into one big parallel loop 

	// 16bit
	vector<short*> prev_depth_ptrs;

	short* background_ptr = background_.ptr<short>();
	short* background_timer_ptr = background_timer_.ptr<short>();
	short* backgr_average_ptr = backgr_average_.ptr<short>();

	short* depth_in_ptr = depth_in_copy.ptr<short>();

	// reverse order of circular buffer so that with i+1 we go back in time
	for (int image_index = prev_images_.size() - 1; image_index >= 0; --image_index)
	{
		prev_depth_ptrs.push_back(prev_images_[image_index].ptr<short>());
	}

	const int numPixels = depth_in_copy.rows * depth_in_copy.cols;
	

	depth_out = Scalar(0);
	short* depth_out_ptr = depth_out.ptr<short>();

	Vec3b* color_in_ptr	 = color_in.ptr<Vec3b>();
	Vec3b* color_out_ptr = color_out.ptr<Vec3b>();

	// local copies so the values don't change during execution 
	const float runaverage_speed = s_.runaverage_speed;
	const float background_timeout = s_.background_timeout;

	const int num_previous_imgs = prev_depth_ptrs.size();

	const float min_dist_back_to_average = s_.min_dist_back_to_average;
	const float max_dist_back_to_average = s_.max_dist_back_to_average;
	const float target_dist_back_to_average = ( min_dist_back_to_average + max_dist_back_to_average) / 2;
	const short numeric_max = ( numeric_limits<unsigned short>::max )();

	#ifdef PC_PERF_MARK
		span *loop_span = new span( *marker_series_, 1,  L"Loop" );
	#endif // PC_PERF_MARK

	for (int pixel_index = 0; pixel_index < numPixels; ++pixel_index, ++depth_out_ptr, ++color_out_ptr,
		++depth_in_ptr, ++color_in_ptr, ++background_ptr,
		++background_timer_ptr, ++backgr_average_ptr)
	{
		// set output to black in case none of our following if statements is true
		// and depth_out and color_out is not reset from last loop

		*depth_out_ptr = 0;
		*color_out_ptr = Vec3b(0, 0, 0);

		short depth = *depth_in_ptr;

		// check if there is some numeric max areas left that were not replaced by erosion
		// and set them back to zero
		if (depth == numeric_max)
		{
			depth = 0;
		}

		// if pixel is zero check if it was not zero in the previous images
		if (depth == 0)
		{
			for (int j = 0; j < num_previous_imgs; ++j)
			{
				short prev_depth = prev_depth_ptrs[j][pixel_index];
				if (prev_depth > 0)
				{
					depth = prev_depth;
					break;
				}
			}
		}


		if (depth != 0)
		{

			if (update_backgr_)
			{
				*backgr_average_ptr = (1.f - runaverage_speed) * (*backgr_average_ptr) + runaverage_speed * depth;
				
				const short diff = *backgr_average_ptr - *background_ptr;


				// check if new value is far enough from current background model and 
				// increment timer, else reset timer 
				if( diff > max_dist_back_to_average || min_dist_back_to_average > diff )
				{
					++(*background_timer_ptr);
				}
				else {
					*background_timer_ptr = 0;
				}

				// if timer passed timeout value: something different to our current background
				// model is staying long enough to be added to our background model 
				if (*background_timer_ptr > background_timeout)
				{
					*background_ptr -= target_dist_back_to_average - diff;
					*background_timer_ptr = 0;
				}
			}
			// accept new pixel if it is closer to camera than background 
			if (*background_ptr > depth)
			{
				*depth_out_ptr = depth;
				*color_out_ptr = *color_in_ptr;
			}
		}
	}
	#ifdef PC_PERF_MARK
		delete loop_span;
	#endif // PC_PERF_MAR

	if( num_previous_imgs > 0 ) prev_images_.push_back(depth_in_copy.clone());
	
#ifdef BK_SHOW_DEBUG_IMG
	Mat back_8;
	background_.convertTo( back_8, CV_8U, 255.f / 7000.f, 0 );
	imshow("BKSubstrktrBack", back_8);
	Mat back_avg_8;
	backgr_average_.convertTo( back_avg_8, CV_8U, 255.f/7000.f, 0 );
	imshow( "BKSubstrktrAvg", back_avg_8 );

	Mat back_timer_8;
	background_timer_.convertTo( back_timer_8, CV_8U, 255.f / background_timeout, 0 );
	imshow( "BKSubstrktrTimer", back_timer_8 );
#endif
}

void background_substractor::set_num_background_history(int num)
{
	s_.num_prev_images = num;
	prev_images_.set_capacity(s_.num_prev_images);
}

void background_substractor::save_backgrd(const std::string &path)
{
	imwrite(path, background_);
}

void background_substractor::load_backgrd(const std::string &path)
{
	background_ = imread(path, CV_LOAD_IMAGE_ANYDEPTH );
	background_.copyTo(backgr_average_);
	background_timer_ = Scalar(0);
}

void background_substractor::set_backgrd(int value)
{
	background_ = Scalar(value);
	background_timer_ = Scalar(0);
}

void background_substractor::toggle_update()
{
	update_backgr_ = !update_backgr_;
}
