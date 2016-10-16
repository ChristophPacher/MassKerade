/*!
 * @file	depth_camera_kinect_v1.cpp
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

#include "kinect_v1.h"

using namespace std;


void kinect_v1::set_playback_speed(float sp)
{
	mDevice->setPlaybackSpeed(sp);
}

void kinect_v1::open(boost::filesystem::path &p, bool restart)
{
	lock_guard<mutex> lock( mutex_ );

	try {

		if (restart)
		{
			mDevice->stop();
			mDeviceManager->deleteDevice(mDevice->getDeviceInfo().getUri());
		}

		OpenNI::DeviceOptions op;
		op.enableColor( true );
		op.enableDepth( true );
		op.enableInfrared( false );

		if (p.empty())
		{
			mDevice = mDeviceManager->createDevice( op );

			if( mDevice->getDevice().isImageRegistrationModeSupported( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
				mDevice->getDevice().setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );	
		}
		else
		{
			op.setUri( p.string() );
			mDevice = mDeviceManager->createPlaybackDevice( op );
		}

		mDevice->getDevice().setDepthColorSyncEnabled( true );

		mDevice->connectDepthEventHandler([&](openni::VideoFrameRef frame, const OpenNI::DeviceOptions& deviceOptions){
			lock_guard<mutex> lock( mutex_ );
			depth_temp_.data = (uchar*)frame.getData();
			depth_ = depth_temp_.clone();
			new_depth_ = true;
		});
		mDevice->connectColorEventHandler([&](openni::VideoFrameRef frame, const OpenNI::DeviceOptions& deviceOptions){
			lock_guard<mutex> lock( mutex_ );
			color_temp_.data = (uchar*)frame.getData();
			color_ = color_temp_.clone();
			new_color_ = true;
		});

		openni::VideoMode depthMode = mDevice->getDepthStream().getVideoMode();
		openni::VideoMode colorMode = mDevice->getColorStream().getVideoMode();

		// TODO get this from driver
		props_.color_format = CV_8UC3;
		props_.depth_format = CV_16UC1;

		depth_temp_.create( depthMode.getResolutionY(), depthMode.getResolutionX(), props_.depth_format );
		color_temp_.create( colorMode.getResolutionY(), colorMode.getResolutionX(), props_.color_format );
		depth_.create( depthMode.getResolutionY(), depthMode.getResolutionX(), props_.depth_format );
		color_.create( colorMode.getResolutionY(), colorMode.getResolutionX(), props_.color_format );


		color_ = cv::Scalar( 0 );
		depth_ = cv::Scalar( 0 );

		props_.depthW = depthMode.getResolutionX();
		props_.depthH = depthMode.getResolutionY();
		props_.colorW = colorMode.getResolutionX();
		props_.colorH = colorMode.getResolutionY();

		props_.depth_FOV_h = mDevice->getDepthStream().getHorizontalFieldOfView();
		props_.depth_FOV_v = mDevice->getDepthStream().getVerticalFieldOfView();

		isPlaying_ = true;

	}
	catch (OpenNI::ExcDeviceNotAvailable ex) {
 		cout << ex.what() << endl;
 		return;
	}
}


void kinect_v1::stop()
{
	mDevice->stop();
}

void kinect_v1::start()
{
	mDevice->start();
}

void kinect_v1::toggle_playback()
{
	if (isPlaying_)
	{
		stop();
		isPlaying_ = false;
	} 
	else
	{
		start();
		isPlaying_ = true;
	}
}

kinect_v1::kinect_v1()
{
	model_ = KINECT1;
	mDeviceManager = OpenNI::DeviceManager::create();
	depth_.create( 10, 10, CV_16UC1 );
	color_.create( 10, 10, CV_8UC3 );
	depth_temp_.create( 10, 10, CV_16UC1 );
	color_temp_.create( 10, 10, CV_8UC3 );
}

kinect_v1::~kinect_v1()
{
	mDevice->stop();
}

