/*!
 * @file	openni_reprojector.cpp
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

#include "openni_reprojector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <ppl.h>

#include "pacher_utilities.h"

using namespace cv;

openni_reprojector::openni_reprojector( const openni_reprojector::settings &s) :
s_(s),
x_to_z_(tan(s.h_fov/2.f)*2),
y_to_z_(tan(s.v_fov/2.f)*2),
transform_z_(myUtilities::calcDataRangeTransformFactors(s.depth_min, s.depth_max, 0, 255))
{

}


void openni_reprojector::reproject( cv::Mat &depth_in, cv::Mat &rgb_in, cv::Mat &depth_out, cv::Mat &rgb_out )
{
	// create const local vars to enable compiler optimization of loop 
	const float w = depth_in.cols;
	const float h = depth_in.rows;
	const float w_half = w / 2.f;
	const float h_half = h / 2.f;
	const float scale = s_.scale;
	const float x_to_z = x_to_z_;
	const float y_to_z = y_to_z_;
	const float transform_z_mul = transform_z_.mul;
	const float transform_z_add = transform_z_.add;
	const int numPixels = w * h;


	// 16bit
	short *p_depth = depth_in.ptr<short>();

	depth_out = Scalar(255);
	rgb_out = Scalar(0, 0, 0);
	Vec3b* rgb_p = rgb_in.ptr<Vec3b>();


	for (int i = 0; i < numPixels; ++i) {	
		
		if (p_depth[i] != 0) {

			const float y = int(i / w);
			const int nX = roundf(((i - y * w) / w - 0.5f) * p_depth[i] * x_to_z * scale + w_half);
			const int nY = roundf((y / h - 0.5f) * p_depth[i] * y_to_z * scale + h_half);
			const int nZ = roundf(p_depth[i] * transform_z_mul + transform_z_add);

			uchar &destinationZ = depth_out.at<uchar>(nY, nX);

			if (nZ > 0 && nZ <= 255 &&
				nX >= 0 && nY >= 0 &&
				nX < w && nY < h &&
				destinationZ > nZ)
			{
				destinationZ = nZ;
				rgb_out.at<Vec3b>(nY, nX) = rgb_p[i];
			}
		}
	}

	depth_out = 255 - depth_out;
}

void openni_reprojector::depth_range_min( float val )
{
	transform_z_ = myUtilities::calcDataRangeTransformFactors(s_.depth_min, s_.depth_max, 0, 255);
}

void openni_reprojector::depth_range_max( float val )
{
	transform_z_ = myUtilities::calcDataRangeTransformFactors(s_.depth_min, s_.depth_max, 0, 255);
}
