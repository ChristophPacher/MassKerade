/*!
 * @file	BasicApp.h
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

#ifndef _H_BASICAPP
#define _H_BASICAPP

#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/MayaCamUI.h"
#include <CinderConfig.h>

#include <ppl.h>

#include <boost/filesystem/path.hpp>

#include <mutex>

#include "depth_camera.h"
#include "kinect_v1.h"
#include "topdown_hand_tracker.h"
#include "masskerade_game.h"

class BasicApp : public ci::app::AppNative
{
public:
	void						draw();
	void						keyDown( ci::app::KeyEvent event );
	void						mouseDown( ci::app::MouseEvent event );
	void						mouseDrag( ci::app::MouseEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();

	void						set_target_fps( float fps );

	void						initKinectTrackerGUI();

	void						update();
	void						shutdown();

private:

	enum left_image_mode		{ DEPTH_SENSOR, DEPTH_BAKSUBSTRK, DEPTH_ORTHO };
	enum right_image_mode		{ IMAGE_SENSOR, IMAGE_ORTHO, IMAGE_TRACKER, RGB_TR_ORTHO };
	int	left_depth_image_mode_ = DEPTH_SENSOR;
	int right_color_image_mode_ = IMAGE_SENSOR;

	struct gui_storage_vars{
		ci::vec3 particle_vel_min_max{};
		ci::vec3 particle_split_vel_min_max{};
		ci::vec3 particle_angular_vel_min_max{};
	};


	topdown_hand_tracker		tracker_;
	topdown_hand_tracker::settings s_;

	ci::gl::TextureRef			mColorRightTex, mDepthLeftTex, mColorRight_btm_Tex, mDepthLeft_btm_Tex;
	ci::gl::PboRef				color_right_pbo_, depth_left_pbo_, color_right_btm_pbo_, depth_left_btm_pbo_;

	int rgb_buffer_size_ = 0, depth_buffer_size_ = 0;
	

	ci::params::InterfaceGlRef	mParams;
	bool show_gui = false;
	bool show_tracker_ = false;
	ci::config::ConfigRef		mConfig;

	concurrency::Scheduler* ppl_scheduler_;

	static std::mutex mutex_;


	gui_storage_vars gui_vars_;
	
	// TODO move this and others into gui_vars
	int gui_background_set_to_ = 3600;
	
	float gui_fps_ = 0.f;
	float gui_target_fps_ = 30.f;
	
	ci::MayaCamOrthoUI			mMayaCamera;
	bool maya_cam_enabled_ = false;

	int kinect_version_ = 1;
	int play_file_ = 0;
	int currently_playing_ = 0;
	float playback_speed_ = 1.f;
	bool isPlaying_ = false;
	std::vector<boost::filesystem::path> vids_;

	depth_camera::depht_camera_ref	mDevice;
	kinect_v1::kinect_v1_ptr		kV1_;
	cv::Mat depth_, color_;
	int flip_camera_images_ = -2; // -2 == do nothing, -1 both axis, 0 around x, 1 around y

	masskerade_game::UPtr game_;
};

#endif                  // include guard



