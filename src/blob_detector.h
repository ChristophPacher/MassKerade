/*!
 * @file	blob_detector.h
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

#ifndef _H_BLOB_DETECTOR
#define _H_BLOB_DETECTOR

#define NOMINMAX

#include <list>
#include <opencv2/core/core.hpp>

#include <ppl.h>

#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK

#include "person.h"


class blob_detector
{
public:
	typedef std::vector<cv::Point> contour;
	

	struct settings {
		int close_num = 5;
		float min_size = 50.f;
	};



	blob_detector() = default;
	explicit blob_detector(const blob_detector::settings &s);

	pacher::blobs detect( cv::Mat &depth, cv::Mat &index_mask_out );

	blob_detector::settings settings_;
	

private:
	cv::Mat kernel_shape_;
#ifdef PC_PERF_MARK
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series_;
#endif // PC_PERF_MARK
};


#endif                  // include guard



