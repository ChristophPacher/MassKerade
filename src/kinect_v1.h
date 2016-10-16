/*!
 * @file	depth_camera_kinect_v1.h
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

#ifndef _H_DEPTH_CAMERA_KINECT_V1
#define _H_DEPTH_CAMERA_KINECT_V1

#include "depth_camera.h"

#include <OpenNI.h>
#include "Cinder-OpenNI.h"

#include <boost/filesystem/path.hpp>




class kinect_v1 : public depth_camera
{
public:
	typedef std::shared_ptr<kinect_v1> kinect_v1_ptr;
	kinect_v1();

	~kinect_v1();

	void virtual start() override;

	void virtual stop() override;

	void open( boost::filesystem::path &p = boost::filesystem::path(), bool restart = false );

	void set_playback_speed(float sp);

	void toggle_playback();
	
	
private:
	OpenNI::DeviceRef			mDevice;
	OpenNI::DeviceManagerRef	mDeviceManager;
	bool isPlaying_ = false;

};


#endif                  // include guard



