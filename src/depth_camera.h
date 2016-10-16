/*!
 * @file	depth_camera.h
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

#ifndef _H_DEPTH_CAMERA
#define _H_DEPTH_CAMERA

#include <opencv2/core/core.hpp>
#include <memory>
#include <mutex>


class depth_camera
{
public:
	typedef std::shared_ptr<depth_camera> depht_camera_ref;

	enum camera_model { KINECT1, KINECT2, NUM_MODELS };

	struct properties{
		int depthW = 0; 
		int depthH = 0;
		int colorW = 0; 
		int colorH = 0;
		float depth_FOV_v = 0.f;
		float depth_FOV_h = 0.f;
		int color_format = CV_8UC1; 
		int depth_format = CV_8UC1;
	};
	void virtual start() = 0;
	void virtual stop() = 0;

	bool new_images();
	void get_images( cv::Mat &rgb, cv::Mat &depth, int flip );

	properties virtual get_properties(){ return props_; };
	camera_model virtual get_model(){ return model_; };


	
protected:
	cv::Mat		depth_, depth_temp_, color_, color_temp_;
	std::mutex mutex_;
	camera_model model_ = NUM_MODELS;
	properties props_;

	bool new_depth_ = false;
	bool new_color_ = false;

};


#endif                  // include guard



