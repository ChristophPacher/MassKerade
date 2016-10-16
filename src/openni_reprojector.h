/*!
 * @file	openni_reprojector.h
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

#ifndef _H_OPENNI_REPROJECTOR
#define _H_OPENNI_REPROJECTOR

#include <opencv2/core/core.hpp>

#include "pacher_utilities.h"

class openni_reprojector
{
public:
	struct settings 
	{
		float h_fov = 0; 
		float v_fov = 0;
		float scale = 1.f;
		float depth_min = 2500.f;
		float depth_max = 5500.f;
	};

	openni_reprojector() = default;
	explicit openni_reprojector( const openni_reprojector::settings &s);

	void reproject(cv::Mat &depth_in, cv::Mat &rgb_in, cv::Mat &depth_out, cv::Mat &rgb_out);
	
	float scale() const { return s_.scale; }
	void scale(float val) { s_.scale = val; }

	void depth_range_min(float val);
	void depth_range_max(float val);

private:

	openni_reprojector::settings s_;
	float	x_to_z_, y_to_z_;

	myUtilities::TransformFactors transform_z_;
};


#endif                  // include guard



