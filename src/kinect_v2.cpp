/*!
 * @file	kinect_v2.cpp
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

#include "kinect_v2.h"


using namespace std;

void kinect_v2::stop()
{
	if ( started_ )
	{
		mDevice->stop();
		started_ = false;
	}
}

void kinect_v2::start()
{
	if (!started_)
	{
		mDevice->start();
		started_ = true;
	}
}

kinect_v2::~kinect_v2()
{
	stop();
}

kinect_v2::kinect_v2()
{
	model_ = KINECT2;
	mDevice = Kinect2::Device::create();
	mDevice->start();
	started_ = true;

	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame& frame)
	{
		lock_guard<mutex> lock(mutex_);
		depth_temp_.data = (uchar*)frame.getChannel().getData();
		depth_ = depth_temp_.clone();
		new_depth_ = true;
	});

	mDevice->connectColorEventHandler([&](const Kinect2::ColorFrame& frame)
	{
		lock_guard<mutex> lock(mutex_);
		color_temp_.data = (uchar*)frame.getSurface().getData();
		color_ = color_temp_.clone();
		new_color_ = true;
	});


	KCBFrameDescription dsc;
	KCBGetDepthFrameDescription(mDevice->getHandle(), &dsc);

	props_.depthW = dsc.width;
	props_.depthH = dsc.height;
	// TODO get this from driver
	props_.color_format = CV_8UC4;
	props_.depth_format = CV_16UC1;

	depth_.create( props_.depthH, props_.depthW, props_.depth_format );
	depth_temp_.create( props_.depthH, props_.depthW, props_.depth_format );
	props_.depth_FOV_h = dsc.horizontalFieldOfView;
	props_.depth_FOV_v = dsc.verticalFieldOfView;




	KCBGetColorFrameDescription(mDevice->getHandle(), ColorImageFormat_Bgra, &dsc);

	props_.colorW = dsc.width;
	props_.colorH = dsc.height;

	color_.create( props_.colorH, props_.colorW, props_.color_format );
	color_temp_.create( props_.colorH, props_.colorW, props_.color_format );
	color_ = cv::Scalar( 0 );
	depth_ = cv::Scalar( 0 );

	new_color_ = new_depth_ = false;
}
