/*!
 * @file	local_maxima.h
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

#ifndef _H_HEAD_DETECTOR
#define _H_HEAD_DETECTOR

#include <opencv2\core\core.hpp>
#include <mutex>
#include <concurrent_vector.h>

class local_maxima
{
public:
	struct PointValue{
		PointValue() = default;
		PointValue( float x, float y, int z ) : p{ x, y }, v{z} {};
		cv::Point2f p = {0.f, 0.f};
		int v = -1;
	};

	struct settings{
		float image_scale = 1.f; 
		int maximum_search_radius = 50;
		int image_width = 0; 
		int min_diff = 5;
		int maximum_blob_range = 10;
	};

	local_maxima() = default;
	explicit local_maxima(const local_maxima::settings &s);
	std::vector<local_maxima::PointValue> detect(const cv::Mat &depth, const cv::Mat &mask );

	void set_search_radius( int maximum_search_radius );

	void set_image_scale( float scale );
	local_maxima::settings settings_;
	
protected:

private:
	int is_maximum( int index, cv::Mat depth_bordered, const std::vector<int>& kernel_offsets );
	void init_kernel( std::vector<int> &kernel_offsets, int &kernel_half, int search_window, float scale );
	PointValue search_maximum( int x, int y, cv::Mat depth_bordered, const std::vector<int>& kernel_offsets, int border );
	
	PointValue get_maximum_blob_centroid( local_maxima::PointValue m, 
		cv::Mat depth_bordered, const std::vector<int>& kernel_offsets, int border );

	static std::mutex mutex_;

	concurrency::concurrent_vector<PointValue> maxima_cncrr;

	int k_half_ = 0;
	int k_scaled_half_ = 0;
	std::vector<int> kernel_pixel_offsets_, kernel_pixel_offsets_scaled_;
	cv::Mat debugout;

};


#endif                  // include guard



