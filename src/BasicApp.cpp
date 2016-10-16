/*!
 * @file	BasicApp.cpp
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
//#include <vld.h> 
#include "BasicApp.h"

// TODO remove cinder_opencv from solution when not needed when finished

#include <exception>


#include "AntTweakBar.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include <boost/assign/std/vector.hpp> 
#include "kinect_v2.h"
#include "kinect_v1.h"
#include "OutputDebugStringBuf.h"


	
using namespace ci;
using namespace ci::app;
using namespace std;
using namespace concurrency;

std::mutex BasicApp::mutex_{};


void BasicApp::update()
{
	lock_guard<mutex> lock( mutex_ );

	

	if( kinect_version_ == 1 && mDevice->new_images() )
	{

		mDevice->get_images( color_, depth_, flip_camera_images_ );
		game_->update( tracker_.track( depth_, color_ ) );
	}
	else
	{
		game_->update();
	}


	if (show_gui || show_tracker_)
	{
		switch( left_depth_image_mode_ )
		{
		case BasicApp::DEPTH_SENSOR:
		{
			cv::Mat conv, flip;
			auto t = tracker_.reprojector_.Transform_Z();
			tracker_.depth_backremoved_.convertTo( conv, CV_8U, t.mul, t.add );
			cv::flip( conv, flip, 0 );
			depth_left_pbo_->bufferSubData( 0, depth_buffer_size_, flip.data );
			mDepthLeftTex->update( depth_left_pbo_, GL_RED, GL_UNSIGNED_BYTE );

			cv::flip( tracker_.depth_ortho_, flip, 0 );
			depth_left_btm_pbo_->bufferSubData( 0, depth_buffer_size_, flip.data );
			mDepthLeft_btm_Tex->update( depth_left_btm_pbo_, GL_RED, GL_UNSIGNED_BYTE );

			break;
		}
		case BasicApp::DEPTH_BAKSUBSTRK:
		{
			cv::Mat conv, flip;
			auto t = tracker_.reprojector_.Transform_Z();
			tracker_.depth_backremoved_.convertTo( conv, CV_8U, t.mul, t.add );
			cv::flip( conv, flip, 0 );
			depth_left_pbo_->bufferSubData( 0, depth_buffer_size_, flip.data );
			mDepthLeftTex->update( depth_left_pbo_, GL_RED, GL_UNSIGNED_BYTE );
			break;
		}
		case BasicApp::DEPTH_ORTHO:
		{
			cv::Mat flip;
			//cv::flip( tracker_.depth_ortho_, flip, 0 );
			cv::flip( tracker_.draw_persons_info_depth(true), flip, 0 );
			depth_left_pbo_->bufferSubData( 0, depth_buffer_size_, flip.data );
			mDepthLeftTex->update( depth_left_pbo_, GL_RED, GL_UNSIGNED_BYTE );
			break;
		}
		default:
			break;
		}
	
		switch( right_color_image_mode_ )
		{
		case BasicApp::IMAGE_SENSOR:
		{
			cv::Mat flip;
			cv::flip( color_, flip, 0 );
	
			color_right_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
			mColorRightTex->update( color_right_pbo_, GL_RGB, GL_UNSIGNED_BYTE );


			cv::flip( tracker_.draw_persons_info( true ), flip, 0 );
			color_right_btm_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
			mColorRight_btm_Tex->update( color_right_btm_pbo_, GL_BGR, GL_UNSIGNED_BYTE );

			break;
		}
		case BasicApp::IMAGE_ORTHO:
		{
			cv::Mat flip;
			cv::flip( tracker_.color_ortho_, flip, 0 );
			color_right_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
			mColorRightTex->update( color_right_pbo_, GL_BGR, GL_UNSIGNED_BYTE );
			break;
		}
		case BasicApp::IMAGE_TRACKER:
		{
			cv::Mat flip;
			cv::flip( tracker_.draw_persons_info( false ), flip, 0 );
			color_right_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
			mColorRightTex->update( color_right_pbo_, GL_RGB, GL_UNSIGNED_BYTE );
			break;
		}
		case BasicApp::RGB_TR_ORTHO:
		{
			cv::Mat flip;
			cv::flip( tracker_.draw_persons_info( true ), flip, 0 );
			color_right_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
			mColorRightTex->update( color_right_pbo_, GL_BGR, GL_UNSIGNED_BYTE );
			break;
		}
		default:
			break;
		}
	}
	else
	{
		cv::Mat flip;
		cv::flip( tracker_.color_ortho_masked_, flip, 0 );
		color_right_pbo_->bufferSubData( 0, rgb_buffer_size_, flip.data );
		mColorRightTex->update( color_right_pbo_, GL_BGR, GL_UNSIGNED_BYTE );
	}

	
	gui_fps_ = getAverageFps();

}

void BasicApp::draw()
{
	lock_guard<mutex> lock( mutex_ );

	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::viewport(vec2(0), toPixels(getWindowSize()));
	gl::clear( Colorf::black() );

	gl::setMatricesWindow(toPixels(getWindowSize()));
	
	
	if (show_tracker_)
	{
		int window_w_half = getWindowBounds().getWidth() / 2;
		int depth_h = window_w_half / mDepthLeftTex->getAspectRatio();
		int color_h = window_w_half / mColorRightTex->getAspectRatio();


		gl::draw( mDepthLeftTex, mDepthLeftTex->getBounds(), Area( 0, 0, window_w_half, depth_h ) );
		gl::draw( mColorRightTex, mColorRightTex->getBounds(), Area( window_w_half, 0, getWindowBounds().getWidth(), color_h ) );

		gl::draw( mDepthLeft_btm_Tex, mDepthLeft_btm_Tex->getBounds(), Area( 0, depth_h, window_w_half, getWindowBounds().getHeight() ) );
		gl::draw( mColorRight_btm_Tex, mColorRight_btm_Tex->getBounds(), Area( window_w_half, color_h, getWindowBounds().getWidth(), getWindowBounds().getHeight() ) );
	}
	else
	{
		//game_->draw();

		gl::draw( mColorRightTex, mColorRightTex->getBounds(),
			Area::proportionalFit( mColorRightTex->getBounds(), getWindowBounds(), true, true ) );

		//gl::draw( mDepthLeftTex, mDepthLeftTex->getBounds(),
		//	Area::proportionalFit( mColorRightTex->getBounds(), getWindowBounds(), true, true ) );
		
	}

	if( show_gui )
	{
		mParams->draw();
	}
}

void BasicApp::keyDown( KeyEvent event )
{
	lock_guard<mutex> lock( mutex_ );

	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_g:
		show_gui = !show_gui;
		break;
	case KeyEvent::KEY_t:
		show_tracker_ = !show_tracker_;
		break;
	case KeyEvent::KEY_p:
		game_->add_particle();
		break;
	case KeyEvent::KEY_d:
		game_->toogle_draw_debug();
		break;
	case KeyEvent::KEY_1:
		game_->all_players_get_particle();
		break;
	case KeyEvent::KEY_k:
		game_->reset();
		break;
	case KeyEvent::KEY_s:
		game_->player_shoot_particle();
		break;
	//case KeyEvent::KEY_f:
	//	setFullScreen( !isFullScreen() );
	//	break;
	}

	if (mDevice->get_model() == depth_camera::KINECT1 && play_file_)
	{
		switch (event.getCode()) {
		case KeyEvent::KEY_LEFT:
			playback_speed_ -= 0.01f;
			kV1_->set_playback_speed(playback_speed_);
			break;
		case KeyEvent::KEY_RIGHT:
			playback_speed_ += 0.01f;
			kV1_->set_playback_speed(playback_speed_);
			break;
		case KeyEvent::KEY_UP:
			playback_speed_ = 1.f;
			kV1_->set_playback_speed(playback_speed_);
			break;
		case KeyEvent::KEY_DOWN:
			playback_speed_ = 0.01f;
			kV1_->set_playback_speed(playback_speed_);
			break;
		case KeyEvent::KEY_SPACE:
			kV1_->toggle_playback();
			tracker_.back_sub_.toggle_update();
			break;
		}
	}
}

void BasicApp::mouseDown( ci::app::MouseEvent event )
{
	if (maya_cam_enabled_)
	{
		mMayaCamera.setCurrentCam( tracker_.reprojector_.cam_ );
		mMayaCamera.mouseDown( event.getPos() );
	}
}

void BasicApp::mouseDrag( ci::app::MouseEvent event )
{
	if ( maya_cam_enabled_ )
	{
		mMayaCamera.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
		tracker_.reprojector_.cam_ = mMayaCamera.getCamera();
	}
}



void BasicApp::prepareSettings( Settings* settings )
{
	settings->setFrameRate( 30.0f );
	settings->setWindowSize( 1440, 1080 );
	settings->setTitle("Masskerade");
	settings->setWindowPos( 0, 0 );
	settings->setBorderless(true);
	//settings->setFullScreen();
	//XmlTree doc( loadFile( getAssetPath( "config.xml" ) ) );
	//XmlTree node;
	//int fullscreen = node.getChild( "fullscreen" ).getValue<int>();
	//if( fullscreen == 1 )
	//{
		//settings->setFullScreen( true );
	//}
}

void BasicApp::setup()
{
#ifdef _WIN32
	// connect streams to console
	static OutputDebugStringBuf<char> charDebugOutput;
	std::cerr.rdbuf( &charDebugOutput );
	std::clog.rdbuf( &charDebugOutput );
	std::cout.rdbuf( &charDebugOutput );

	static OutputDebugStringBuf<wchar_t> wcharDebugOutput;
	std::wcerr.rdbuf( &wcharDebugOutput );
	std::wclog.rdbuf( &wcharDebugOutput );
	std::wcout.rdbuf( &wcharDebugOutput );
#endif
	using namespace std;
	using namespace cv;

	XmlTree doc(loadFile(getAssetPath("config.xml")));
	XmlTree node;
	node = doc.getChild("general");
	kinect_version_ = node.getChild("kinect").getValue<int>();
	play_file_ = node.getChild( "playback_1_or_live_0" ).getValue<int>();


	SetPriorityClass( GetCurrentProcess(), REALTIME_PRIORITY_CLASS );
	SchedulerPolicy my_policy(3,
		MinConcurrency, 4,
		MaxConcurrency, 4,
		ContextPriority, THREAD_PRIORITY_HIGHEST);

	cv::setNumThreads( 4 );

	ppl_scheduler_ = Scheduler::Create(my_policy);
	ppl_scheduler_->Attach();


	initKinectTrackerGUI();

}


void BasicApp::shutdown()
{
	ppl_scheduler_->Release();
	mDevice->stop();
}

void BasicApp::initKinectTrackerGUI()
{
	if (kinect_version_ == 1)
	{
		mDevice = make_shared<kinect_v1>();
	}
	else {
		mDevice = make_shared<kinect_v2>();
	}


	if (mDevice->get_model() == depth_camera::KINECT1)
	{
		kV1_ = static_pointer_cast<kinect_v1>(mDevice);

		fs::path vids_path = getAssetPath("vids");
		if (fs::is_directory(vids_path) && !vids_path.empty())
		{
			for (fs::directory_iterator it(vids_path); it != fs::directory_iterator(); ++it)
			{
				vids_.push_back(it->path());
			}
		}

		if ( vids_.size() > 0 && play_file_ == 1 )
		{
			kV1_->open(vids_[currently_playing_]);
		}
		else
		{
			kV1_->open();
		}
	}

	auto props = mDevice->get_properties();
	
	rgb_buffer_size_ = 3 * props.colorW * props.colorH;
	depth_buffer_size_ = props.depthW * props.depthH;

	masskerade_game::settings g_s;
	g_s.w_tracker_input = props.depthW;
	g_s.h_tracker_input = props.depthH;
	g_s.w_window = getWindowWidth();
	g_s.h_window = getWindowHeight();
	g_s.camera_fov_v = props.depth_FOV_v;
	g_s.light_pos = vec3(g_s.w_tracker_input / 2.f + g_s.off_screen_border, 
						 g_s.h_tracker_input / 2.f + g_s.off_screen_border,
						 0.f);

	// since suBox2D uses unique ptrs we cannot construct
	// our game class on the stack as a cinder app member
	// since we want to settings construct a new 
	// game now but cannot assign it here since unique ptrs can only be 
	// moved and this would mean we need to implement move assignment ops
	// for all the wrapper classes in suBox2d
	game_ = masskerade_game::UPtr( new masskerade_game( g_s ) );
	
	using namespace ci::gl;
	color_right_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, rgb_buffer_size_, NULL, GL_STREAM_DRAW );
	depth_left_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, depth_buffer_size_, NULL, GL_STREAM_DRAW );

	color_right_btm_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, rgb_buffer_size_, NULL, GL_STREAM_DRAW );
	depth_left_btm_pbo_ = Pbo::create( GL_PIXEL_UNPACK_BUFFER, depth_buffer_size_, NULL, GL_STREAM_DRAW );

	
	auto depthForm = Texture::Format();
	depthForm.setInternalFormat(GL_R8);
	depthForm.setPixelDataFormat(GL_RED);
	depthForm.setPixelDataType(GL_UNSIGNED_BYTE);
	depthForm.enableMipmapping();
	depthForm.setMagFilter(GL_LINEAR);
	depthForm.setMinFilter(GL_LINEAR_MIPMAP_LINEAR);
	mDepthLeftTex = Texture::create(props.depthW, props.depthH, depthForm);
	mDepthLeft_btm_Tex = Texture::create( props.depthW, props.depthH, depthForm );

	auto colorForm = Texture::Format();
	colorForm.setInternalFormat(GL_RGB8);
	colorForm.setPixelDataFormat(GL_BGR);
	if (mDevice->get_model() == depth_camera::KINECT2)
	{
		colorForm.setInternalFormat(GL_RGBA8);
		colorForm.setPixelDataFormat(GL_BGRA);
	}
	colorForm.setPixelDataType(GL_UNSIGNED_BYTE);
	colorForm.enableMipmapping();
	colorForm.setMagFilter(GL_LINEAR);
	colorForm.setMinFilter(GL_LINEAR_MIPMAP_LINEAR);
	mColorRightTex = Texture::create(props.colorW, props.colorH, colorForm);
	mColorRight_btm_Tex = Texture::create( props.colorW, props.colorH, colorForm );

	color_.create( props.colorH, props.colorW, props.color_format );
	depth_.create( props.depthH, props.depthW, props.depth_format );

	mDevice->start();
	isPlaying_ = true;

	if (mDevice) {


		s_.depthH = props.depthH;
		s_.depthW = props.depthW;
		s_.rgbH = props.colorH;
		s_.rgbW = props.colorW;
		s_.fov_h = props.depth_FOV_h;
		s_.fov_v = props.depth_FOV_v;
		
		// 3 chars = 3 bytes
		s_.rgb_elem_size = 3;
		// short is two bytes large
		s_.depth_elem_size = 2;

		s_.maxima.image_width = props.depthW;

		s_.background_sub.w = props.depthW;
		s_.background_sub.h = props.depthH;
		s_.background_sub.type = props.depth_format;

		s_.reprojector.h_fov = s_.fov_h;
		s_.reprojector.v_fov = s_.fov_v;
		s_.reprojector.depth_h = props.depthH;
		s_.reprojector.depth_w = props.depthW;

		tracker_ = topdown_hand_tracker(s_);


		mParams = params::InterfaceGl::create( getWindow(), "Main parameters", toPixels( ivec2( 300, getWindowBounds().getHeight() ) ) );
		mConfig = config::Config::create(mParams);

		mParams->setPosition( ivec2( getWindowBounds().getCenter().x -150, 0 ) );

		mParams->addParam("Current FPS", &gui_fps_).optionsStr("readonly=true");
		mConfig->addParam( "Target FPS", &gui_target_fps_ )
			.updateFn( [ this ] {
			set_target_fps( gui_target_fps_ );
		} ).min( 30 ).max( 999 ).precision( 0 ).step( 1 );

		mConfig->addParam("Kinect", &kinect_version_)
			.updateFn( [ this ] {} ).min( 1 ).max( 2 ).step( 1 ).group( "Camera" );


		if (mDevice->get_model() == depth_camera::KINECT1)
		{
#ifdef _DEBUG
			playback_speed_ = 0.2f;
#endif
			//TODO implement kinect switch function if really needed?
			mParams->addParam("Playback Speed", &playback_speed_)
				.updateFn( [ this ] { kV1_->set_playback_speed( playback_speed_ ); } ).min( 0.001f ).max( 5.f ).step( 0.001 ).group( "Camera" );

			kV1_->set_playback_speed( playback_speed_ );

			mParams->addParam("Video", &currently_playing_)
				.updateFn([&] {
				lock_guard<mutex> lock( BasicApp::mutex_ );
				tracker_.reset();
				kV1_->open(vids_[currently_playing_], true);
				mDevice->start();
				kV1_->set_playback_speed( playback_speed_ );
			} ).min( 0 ).max( vids_.size() - 1 ).step( 1 ).group( "Camera" );

			mConfig->addParam( "Playback 1 or live 0", &play_file_ )
				.min( 0 ).max( 1 ).precision( 0 ).step( 1 ).group( "Camera" );
		}

		mConfig->addParam( "Flip Camera Image", &flip_camera_images_ )
			.min( -2 ).max( 1 ).precision( 0 ).step( 1 ).group( "Camera" );



		mConfig->addParam( "Erode Roi x", &tracker_.back_sub_.s_.roi_x )
			.min( 0 ).max( props.depthW ).precision( 0 ).step( 1 ).group( "Background Subtraction" );
		mConfig->addParam( "Erode Roi y", &tracker_.back_sub_.s_.roi_y )
			.min( 0 ).max( props.depthH ).precision( 0 ).step( 1 ).group( "Background Subtraction" );
		mConfig->addParam( "Erode Roi w", &tracker_.back_sub_.s_.roi_w )
			.min( 0 ).max( props.depthW ).precision( 0 ).step( 1 ).group( "Background Subtraction" );
		mConfig->addParam( "Erode Roi h", &tracker_.back_sub_.s_.roi_h )
			.min( 0 ).max( props.depthH ).precision( 0 ).step( 1 ).group( "Background Subtraction" );

		mConfig->addParam( "Erode iterations", &tracker_.back_sub_.s_.erode_iterations )
			.min( 0 ).max( 10 ).precision( 0 ).step( 1 ).group( "Background Subtraction" );
		mConfig->addParam( "Erode Kernel Size", &tracker_.back_sub_.s_.erode_size )
			.min( 1 ).max( 5 ).precision( 0 ).step( 2 ).group( "Background Subtraction" );
		mConfig->addParam("MedianBlur iterations", &tracker_.back_sub_.s_.median_iterations)
			.min(0).max(10).precision(0).step(1).group("Background Subtraction");
		mConfig->addParam("MedianBlur Kernel Size", &tracker_.back_sub_.s_.median_size)
			.min(1).max(5).precision(0).step(2).group("Background Subtraction");
		mConfig->addParam("Num History Images", &tracker_.back_sub_.s_.num_prev_images)
			.updateFn([this] {
			lock_guard<mutex> lock(mutex_);
			tracker_.back_sub_.set_num_background_history(tracker_.back_sub_.s_.num_prev_images);
		}).min(0).max(36000).precision(0).step(1).group("Background Subtraction");

		mConfig->addParam("Timeout", &tracker_.back_sub_.s_.background_timeout)
			.min(1).max(36000).precision(0).step(180).group("Background Subtraction");
		mConfig->addParam("Learn Rate", &tracker_.back_sub_.s_.runaverage_speed)
			.min(0.00001f).max(1.f).precision(6).step(0.00001f).group("Background Subtraction");
		mConfig->addParam("Min Distance to Average", &tracker_.back_sub_.s_.min_dist_back_to_average)
			.min(2.f).max(10000.f).precision(0).step(1.f).group("Background Subtraction");
		mConfig->addParam("Max Distance to Average", &tracker_.back_sub_.s_.max_dist_back_to_average)
			.min(2.f).max(10000.f).precision(0).step(1.f).group("Background Subtraction");

		mParams->addButton("Save Backgrnd", [&](){
			lock_guard<mutex> lock(mutex_);
			tracker_.back_sub_.save_backgrd((getAssetPath("") / "background.tif").string());
		});
		mParams->setOptions("Save Backgrnd", "group='Background Subtraction'");

		mParams->addButton("Save Depth", [&](){
			lock_guard<mutex> lock(mutex_);
			imwrite((getAssetPath("") / "depth.tif").string(), depth_);
		});
		mParams->setOptions("Save Depth", "group='Background Subtraction'");

		mParams->addButton("Load Backgrnd", [&](){
			lock_guard<mutex> lock(mutex_);
			tracker_.back_sub_.load_backgrd(getAssetPath("background.tif").string());
		});
		mParams->setOptions("Load Backgrnd", "group='Background Subtraction'");

		mConfig->addParam("Backgrnd set to", &gui_background_set_to_)
			.min(0).max(10000).precision(0).step(1).group("Background Subtraction");

		mParams->addButton("Reset Backgrnd", [&](){
			lock_guard<mutex> lock(mutex_);
			tracker_.back_sub_.set_backgrd(gui_background_set_to_);
		});
		mParams->setOptions("Reset Backgrnd", "group='Background Subtraction'");



		mConfig->addParam("Enable Mayacam", &maya_cam_enabled_)
			.group("Reprojector");
		mParams->addButton("Reset Camera", [&](){
			tracker_.reprojector_.cam_.setEyePoint(vec3(0.f, 0.f, 20.f));
			tracker_.reprojector_.cam_.setCenterOfInterestPoint(vec3(0.f, 0.f, 0.f));
		});
		mParams->setOptions("Reset Camera", "group='Reprojector'");

		mConfig->addParam("Camera Width", &s_.reprojector.w_camera)
			.updateFn([this] { tracker_.reprojector_.Camera_width(s_.reprojector.w_camera); })
			.min(0.01).max(10000).precision(2).step(1).group("Reprojector");

		mConfig->addParam("Minimum Depth", &s_.reprojector.depth_min)
			.updateFn([this] { tracker_.reprojector_.depth_range_min(s_.reprojector.depth_min); })
			.min(0).max(10000).precision(1).step(1).group("Reprojector");

		mConfig->addParam("Maximum Depth", &s_.reprojector.depth_max)
			.updateFn([this] { tracker_.reprojector_.depth_range_max(s_.reprojector.depth_max); })
			.min(0).max(10000).precision(1).step(1).group("Reprojector");

		mConfig->addParam("Mesh Translation", &s_.reprojector.mesh_translation)
			.updateFn([this] { tracker_.reprojector_.Mesh_translation(s_.reprojector.mesh_translation); })
			.group("Reprojector");
		mConfig->addParam("Mesh Rotation", &s_.reprojector.mesh_rotation)
			.updateFn([this] { tracker_.reprojector_.Mesh_rotation(s_.reprojector.mesh_rotation); })
			.group("Reprojector");

		mConfig->addParam("Mesh Threshold", &tracker_.reprojector_.s_.mesh_threshold)
			.group("Reprojector");




		mConfig->addParam("Min Contour Size", &tracker_.blob_detector_.settings_.min_size)
			.min(1).max(10000).precision(0).step(1).group("Blob Detector");
		mConfig->addParam("Contour Closing", &tracker_.blob_detector_.settings_.close_num)
			.min(0).max(50).precision(0).step(1).group("Blob Detector");



		mConfig->addParam( "Image Scale", &s_.maxima.image_scale )
			.updateFn( [ this ] { tracker_.maxima_.set_image_scale( s_.maxima.image_scale ); } )
			.min( 0.001f ).max( 1.f ).precision( 5 ).step( 0.001 ).group( "Local Maxima" );
		mConfig->addParam( "Search Radius", &s_.maxima.maximum_search_radius )
			.updateFn( [ this ] { tracker_.maxima_.set_search_radius( s_.maxima.maximum_search_radius ); } )
			.min( 1 ).max( 500 ).precision( 0 ).step( 1 ).group( "Local Maxima" );
		mConfig->addParam( "Maxima match Radius sq", &tracker_.settings_.maxima_match_radius_sq )
			.min( 1 ).max( 9999999 ).precision( 0 ).step( 10 ).group( "Local Maxima" );
		mConfig->addParam( "Maxima min Diff to Surrounding", &tracker_.maxima_.settings_.min_diff )
			.min( 1 ).max( 9999999 ).precision( 0 ).step( 1 ).group( "Local Maxima" );
		mConfig->addParam( "Maxima Blob value Range", &tracker_.maxima_.settings_.maximum_blob_range )
			.min( 1 ).max( 9999999 ).precision( 0 ).step( 1 ).group( "Local Maxima" );



		mConfig->addParam( "mask erode iterations", &tracker_.feature_detector_.settings_.mask_erode_interations )
			.min( 0 ).max( 20 ).precision( 0 ).step( 1 ).group( "Feature Detector" );
		mConfig->addParam( "Detector Threshold", &s_.feature_detec.detector_threshold )
			.updateFn( [ this ] { tracker_.feature_detector_.detector_threshold( s_.feature_detec.detector_threshold ); } )
			.min( 1 ).max( 5000 ).precision( 0 ).step( 1 ).group( "Feature Detector" );
		mConfig->addParam("match 1 to 2 distance ratio", &tracker_.feature_detector_.settings_.match_1_2_ratio )
			.min(0).max(2).precision(3).step(1).group("Feature Detector");
		mConfig->addParam( "max Descriptor Distance Multi", &tracker_.feature_detector_.settings_.max_descriptor_distance_multi )
			.min( 1 ).max( 5000 ).precision( 2 ).step( 1 ).group( "Feature Detector" );
		mConfig->addParam("max Key point Distance", &tracker_.feature_detector_.settings_.max_keypoint_distance)
			.min( 1 ).max( 5000 ).precision( 0 ).step( 1 ).group( "Feature Detector" );



		mConfig->addParam("Matching Radius sq", &tracker_.settings_.disappeared_match_radius_sq)
			.min(1).max(100000).precision(0).step(1).group("Disappeared Person");
		mConfig->addParam("Seconds Remembered", &tracker_.settings_.disappeared_timeout_s)
			.min(1).max(10000).precision(0).step(1).group("Disappeared Person");



		mConfig->addParam( "New Person min size", &tracker_.settings_.new_person_min_area)
			.min( 1 ).max( 10000 ).precision( 0 ).step( 1 ).group( "New Person" );
		mConfig->addParam( "Max Person Pos Match Distance sq", &tracker_.settings_.pos_match_radius_sq )
			.min( 1 ).max( 10000 ).precision( 0 ).step( 1 ).group( "New Person" );
		mConfig->addParam( "Max Area Diff for match", &tracker_.settings_.area_match_max_diff )
			.min( 1 ).max( 10000 ).precision( 0 ).step( 1 ).group( "New Person" );



		mConfig->addParam( "Height to Head Radius", &tracker_.hand_finder_.s_.height_to_headradius )
			.min( 0.00001 ).max( 10 ).precision( 5 ).step( 0.01 ).group( "Hand Finder" );
		mConfig->addParam( "Min Hand Size", &tracker_.hand_finder_.s_.hands_min_size )
			.min( 10 ).max( 99999 ).precision( 0 ).step( 1 ).group( "Hand Finder" );
		mConfig->addParam( "Min Defect Depth", &tracker_.hand_finder_.s_.defect_min_depth )
			.min( 10 ).max( 99999 ).precision( 0 ).step( 1 ).group( "Hand Finder" );
		mConfig->addParam( "Height to Hand Threshold", &tracker_.hand_finder_.s_.height_to_handthreshold )
			.min( 0.00001 ).max( 10 ).precision( 5 ).step( 0.01 ).group( "Hand Finder" );
	



		mConfig->addParam( "Hands straight angle (rad)", &shoot_gesture::s_.hands_straight_angle_rad )
			.min( 2.7925268 ).max( 3.31612558 ).precision( 6 ).step( 0.001 ).group( "Shoot Gesture" );
		mConfig->addParam( "Delay between Hands extended and Shoot (ms)", 
			&shoot_gesture::s_.extend_to_shoot_delay_ms )
			.min( 1 ).max( 9999999 ).precision( 0 ).step( 1 ).group( "Shoot Gesture" );
		mConfig->addParam( "Max Delay between Hands extended and Shoot (ms)",
			&shoot_gesture::s_.extend_to_shoot_max_delay_ms )
			.min( 1 ).max( 9999999 ).precision( 0 ).step( 1 ).group( "Shoot Gesture" );



		mParams->addSeparator("GUI");


		mConfig->addParam( "Left Image", { "DEPTH_SENSOR", "DEPTH_BAKSUBSTRK", "DEPTH_ORTHO" }, 
			&left_depth_image_mode_ );
		mConfig->addParam( "Right Image", { "IMAGE_SENSOR", "IMAGE_ORTHO", "IMAGE_TRACKER", "RGB_TR_ORTHO" }, 
			&right_color_image_mode_ );



		// TODO use bind instead of lambda where possible?
		mConfig->addParam( "Light Pos", &game_->s_.light_pos ).group( "Game" );
		mConfig->addParam( "Point Light (0/1)", &game_->s_.point_light )
			.min( 0 ).max( 1 ).precision( 0 ).step( 1 ).group( "Game" );
		mConfig->addParam( "Light Shinyness", &game_->s_.light_shinyness )
			.min( 0 ).max( 999 ).precision( 0 ).step( 1 ).group( "Game" );
		mConfig->addParam( "Light Color", &game_->s_.light_color ).group( "Game" );
		
		mConfig->addParam( "Ambient Color", &game_->s_.ambient_color ).group( "Game" );
		mConfig->addParam( "max Particles", &game_->s_.max_particles )
			.min( 0 ).max( 999 ).precision( 0 ).step( 1 ).group( "Game" );


		mConfig->addParam( "Player radius", &mass_player::s_.player_radius )
			.updateFn( [ this ] { game_->set_player_radius( mass_player::s_.player_radius ); } )
			.min( 0.01f ).max( 999.f ).precision( 2 ).step( 0.01f ).group( "Player" );
		mConfig->addParam( "Orbit Joint Hz, Damp", &mass_player::s_.joint_Hz_Damp )
			.updateFn( [ this ] { game_->set_player_orbit_joint( mass_player::s_.joint_Hz_Damp ); } )
			.group( "Player" );
		mConfig->addParam( "first orbit radius", &mass_player::s_.first_orbit )
			.updateFn( [ this ] { game_->set_player_first_orbit( mass_player::s_.first_orbit ); } )
			.min( 0.01f ).max( 9999.f ).precision( 2 ).step( 0.01f ).group( "Player" );
		mConfig->addParam( "orbit radius incr", &mass_player::s_.orbit_incr )
			.updateFn( [ this ] { game_->set_player_orbit_incr( mass_player::s_.orbit_incr ); } )
			.min( 0.01f ).max( 9999.f ).precision( 2 ).step( 0.01f ).group( "Player" );
		mConfig->addParam( "max particles collected", &game_->s_.player_max_electrons )
			.min( 1 ).max( 26 ).precision( 0 ).step( 1 ).group( "Player" );
		mConfig->addParam( "protons on start", &game_->s_.player_new_protons )
			.min( 1 ).max( 26 ).precision( 0 ).step( 1 ).group( "Player" );
		mConfig->addParam( "disappeared timeout s", &game_->s_.disappeared_timeout_s )
			.min( 0 ).max( 26 ).precision( 1 ).step( 1 ).group( "Player" );
		mConfig->addParam( "proton joint Hz Damp Len", &game_->s_.proton_joint_Hz_Damp_len )
			.updateFn( [ this ] { game_->set_player_proton_joint( game_->s_.proton_joint_Hz_Damp_len ); } )
			.group( "Player" );
		mConfig->addParam( "split off stages", &game_->s_.split_off_stages )
			.group( "Player" );


		mConfig->addParam( "particle min max vel", &gui_vars_.particle_vel_min_max )
			.updateFn( [ this ] { game_->set_particle_vel_( vec2( gui_vars_.particle_vel_min_max ) ); } )
			.group( "Particle" );
		mConfig->addParam( "particle angular min max vel", &gui_vars_.particle_angular_vel_min_max )
			.updateFn( [ this ] { game_->set_particle_angluar_vel_( vec2( gui_vars_.particle_angular_vel_min_max ) ); } )
			.group( "Particle" );
		mConfig->addParam( "particle min max split vel", &gui_vars_.particle_split_vel_min_max )
			.updateFn( [ this ] { game_->set_particle_split_vel_( vec2( gui_vars_.particle_split_vel_min_max ) ); } )
			.group( "Particle" );
		mConfig->addParam( "collected speed", &mass_player::s_.collected_speed )
			.updateFn( [ this ] { game_->set_particle_collected_vel_( mass_player::s_.collected_speed ); } )
			.min( 0.01f ).max( 9999.f ).precision( 2 ).step( 0.01f ).group( "Particle" );
		mConfig->addParam( "shot speed", &mass_player::s_.shot_speed )
			.min( 0.01f ).max( 9999.f ).precision( 2 ).step( 0.01f ).group( "Particle" );
		mConfig->addParam( "max shot bounces", &game_->s_.max_shot_bounces )
			.min( 0 ).max( 99 ).precision( 0 ).step( 1 ).group( "Particle" );
		mConfig->addParam( "Particle Radius", &game_->s_.particle_radius )
			.updateFn( [ this ] { game_->set_particle_radius( game_->s_.particle_radius ); } )
			.min( 1 ).max( 999 ).precision( 0 ).step( 1 ).group( "Particle" );
		mConfig->addParam( "Col A", &game_->s_.colorA ).group( "Particle" );
		mConfig->addParam( "Col B", &game_->s_.colorB ).group( "Particle" );
		mConfig->addParam( "Col A collected", &game_->s_.colorA_collected ).group( "Particle" );
		mConfig->addParam( "Col B collected", &game_->s_.colorB_collected ).group( "Particle" );
		mConfig->addParam( "Col A shot", &game_->s_.colorA_shot ).group( "Particle" );
		mConfig->addParam( "Col B shot", &game_->s_.colorB_shot ).group( "Particle" );
		mConfig->addParam( "Col A split off", &game_->s_.colorA_splitoff ).group( "Particle" );
		mConfig->addParam( "Col B split off", &game_->s_.colorB_splitoff ).group( "Particle" );
		


		mConfig->addParam( "proton damp dampAng restituition", &game_->s_.proton_damp_dampang_restitution )
			.updateFn( [ this ] { game_->set_proton_damp_dampang_restituion( game_->s_.proton_damp_dampang_restitution ); } )
			.group( "Proton" );
		mConfig->addParam( "Col A Proton", &game_->s_.colorA_proton ).group( "Proton" );
		mConfig->addParam( "Col B Proton", &game_->s_.colorB_proton ).group( "Proton" );



		mParams->addSeparator("File Actions");
		mParams->addButton("Save Config", [&](){
			mConfig->save(getAssetPath("config.xml"));
		});

		//close groups by default 
		mParams->setOptions( "Background Subtraction", "opened=false" );

		//mParams->hide();

		auto configpath = getAssetPath("config.xml");
		if(!configpath.empty())
			mConfig->load(configpath);


		// update fn is not called when new settings are loaded from xml so we must manually call them
		set_target_fps( gui_target_fps_ );
		tracker_.reprojector_.depth_range_min( s_.reprojector.depth_min );
		tracker_.reprojector_.depth_range_max( s_.reprojector.depth_max );
		tracker_.reprojector_.Mesh_rotation( s_.reprojector.mesh_rotation );
		tracker_.reprojector_.Mesh_translation( s_.reprojector.mesh_translation );
		tracker_.reprojector_.Camera_width( s_.reprojector.w_camera );
		tracker_.back_sub_.set_num_background_history( tracker_.back_sub_.s_.num_prev_images );
		tracker_.maxima_.set_search_radius( s_.maxima.maximum_search_radius );
		tracker_.maxima_.set_image_scale( s_.maxima.image_scale );
		tracker_.feature_detector_.detector_threshold( s_.feature_detec.detector_threshold );
		game_->set_particle_radius( game_->s_.particle_radius );
		game_->set_player_radius( mass_player::s_.player_radius );
		game_->set_player_first_orbit( mass_player::s_.first_orbit );
		game_->set_player_orbit_incr( mass_player::s_.orbit_incr );
		game_->set_player_orbit_joint( mass_player::s_.joint_Hz_Damp );
		game_->set_particle_collected_vel_( mass_player::s_.collected_speed );
		game_->set_particle_vel_( vec2( gui_vars_.particle_vel_min_max ) );
		game_->set_particle_angluar_vel_( vec2( gui_vars_.particle_angular_vel_min_max ) );
		game_->set_particle_split_vel_( vec2( gui_vars_.particle_split_vel_min_max ) );
		game_->set_player_proton_joint( game_->s_.proton_joint_Hz_Damp_len );
		game_->set_proton_damp_dampang_restituion( game_->s_.proton_damp_dampang_restitution );

		auto backgrpath = getAssetPath("background.tif");
		if (!backgrpath.empty())
			tracker_.back_sub_.load_backgrd(getAssetPath("background.tif").string());
	}
}

void BasicApp::set_target_fps( float fps )
{
	game_->set_fps( fps );
	setFrameRate( fps );
}

CINDER_APP_NATIVE(BasicApp, RendererGl)
