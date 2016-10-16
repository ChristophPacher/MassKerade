/*!
 * @file	background_substractor.h
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

#ifndef _H_BACKGROUND_SUBSTRACTOR
#define _H_BACKGROUND_SUBSTRACTOR

#include <opencv2/core/core.hpp>
#include <opencv2/video/background_segm.hpp>
#include <boost/circular_buffer.hpp>
#include <string>

#include <ppl.h>
#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK

class background_substractor
{

public:

	struct settings {
		int w = 0;
		int h = 0;
		int type = 0;
		int median_size = 5;
		int erode_size = 3;
		int median_iterations = 2;
		int erode_iterations = 10;
		int roi_x = 21;
		int roi_y = 20;
		int roi_w = 617 - roi_x;
		int roi_h = 473 - roi_y;
		float runaverage_speed = 0.001f;
		float background_timeout = 30.f * 60.f * 2.f;
		float min_dist_back_to_average = 200;
		float max_dist_back_to_average = 250;
		int num_prev_images = 0;
	};

	background_substractor() = default;
	explicit background_substractor( const background_substractor::settings &s );
	
	void save_backgrd(const std::string &path);
	void load_backgrd(const std::string &path);

	void process(cv::Mat &img_in, cv::Mat &img_out, cv::Mat &color_in, cv::Mat &color_out);
	void set_num_background_history(int num);
	void set_backgrd(int value);
	void toggle_update();

	background_substractor::settings s_;



private:
	bool update_backgr_ = true;
	//cv::Ptr<cv::BackgroundSubtractor> bksubtrktr_;
	boost::circular_buffer<cv::Mat> prev_images_;
	cv::Mat background_, background_timer_, backgr_average_;
#ifdef PC_PERF_MARK
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series_;
#endif // PC_PERF_MARK

};


#endif                  // include guard



