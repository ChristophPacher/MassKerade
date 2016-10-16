/*!
 * @file	openni_reprojector_gpu.cpp
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

#include "openni_reprojector_gpu.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

#include <CinderOpenCV.h>

#include <cinder/gl/GlslProg.h>
#include <cinder/gl/VboMesh.h>
#include <cinder/app/AppNative.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "pacher_utilities.h"
#include "Timer.h"




using namespace cv;
using namespace ci;
using namespace ci::app;
using namespace std;
#ifdef PC_PERF_MARK
using namespace concurrency;
using namespace concurrency::diagnostic;
#endif // PC_PERF_MARK

//#define RP_SHOW_DEBUG_IMG

openni_reprojector_gpu::openni_reprojector_gpu( const openni_reprojector_gpu::settings &s ) :
s_( s ),
x_to_z_( tan( s.h_fov / 2.f ) * 2.f ),
y_to_z_( tan( s.v_fov / 2.f ) * 2.f ),
transform_z_( myUtilities::calcDataRangeTransformFactors( s.depth_min, s.depth_max, 0.f, 255.f ) ),
buffer_size_( 4 * s_.depth_w * s_.depth_h )
{
	Mesh_rotation( s.mesh_rotation );
	Mesh_translation( s.mesh_translation );
	Camera_width( s.w_camera );

	viewport_mat_ = glm::scale( mat4( 1.0f ), vec3( s.depth_w - 1, s.depth_h - 1, 1.f ) );
	viewport_mat_ = glm::translate( viewport_mat_, vec3( 0.5f, 0.5f, 0.f ) );
	viewport_mat_ = glm::scale( viewport_mat_, vec3( 0.5f, 0.5f, 1.f ) );

	copy_buff_ = gl::BufferObj::create( GL_COPY_WRITE_BUFFER, buffer_size_, NULL, GL_STREAM_READ );
	output_buff_ = gl::BufferObj::create( GL_PIXEL_PACK_BUFFER, buffer_size_, NULL, GL_STATIC_COPY );

	gl::Fbo::Format format;
	//format.setSamples( 4 ); // uncomment this to enable 4x antialiasing
	format.depthBuffer();
	fbo_ = gl::Fbo::create( s_.depth_w, s_.depth_h, format );


	auto g = geom::Grid();
	g.size( s_.depth_w, s_.depth_h );
	
	mesh_ = gl::VboMesh::create( g );

	cam_ = CameraOrtho( -s_.depth_w * 0.5f, s_.depth_w * 0.5f, -s_.depth_h * 0.5f, s_.depth_h * 0.5f, 1.f, 100.f );
	cam_.setEyePoint( vec3( 0.f, 0.f, 20.f ) );
	cam_.setCenterOfInterestPoint( vec3( 0.f, 0.f, 0.f ) );
	
	
	try {
		gl::GlslProg::Format format;
		format.vertex( loadAsset( "reproject_vert.glsl" ) )
			.fragment( loadAsset( "reproject_frag.glsl" ) )
			.geometry( loadAsset( "reproject_geom.glsl" ) );

		//format.vertex( loadAsset( "reproject_vert.glsl" ) )
		//	.fragment( loadAsset( "reproject_frag.glsl" ) );

		shader_ = gl::GlslProg::create( format );
	}
	catch ( gl::GlslProgCompileExc exc ){
		cout << exc.what() << endl;
	}

#ifdef PC_PERF_MARK
	marker_series_ = make_shared<marker_series>( L"Reprojector" );
#endif
#ifdef RP_SHOW_DEBUG_IMG
	namedWindow("Reprojector", WINDOW_AUTOSIZE);
#endif // RP_SHOW_DEBUG_IMG
}

// removes key points and rearranges the input key point vector!
std::vector<cv::KeyPoint> openni_reprojector_gpu::reproject( std::vector<cv::KeyPoint> &in, const cv::Mat &depth_in )
{
#ifdef PC_PERF_MARK
	span repro_span{ *marker_series_, L"CPU reproject kps" }; ;
#endif // PC_PERF_MAR
	
	std::vector<cv::KeyPoint> kp_orth;
	std::vector<cv::KeyPoint> kp_out;
	kp_orth.reserve( in.size() );
	kp_out.reserve( in.size() );

	// create const local vars to enable compiler optimization of loop 
	const float w = depth_in.cols;
	const float h = depth_in.rows;
	const float x_max = w - 1.f;
	const float y_max = h - 1.f;
	const float w_half = w / 2.f;
	const float h_half = h / 2.f;

	const float x_to_z = x_to_z_;
	const float y_to_z = y_to_z_;


	mat4 mvpw = viewport_mat_ *
				cam_.getProjectionMatrix() *
				cam_.getViewMatrix() *
				mesh_model_mat_;

	// TODO parallel_for
	// but kps could not be in same order afterwards,  check!
	// perhaps do an inplace transform and mark kps that should be deleted afterwards
	for ( int i = 0, numPoints = in.size(); i < numPoints; ++i ) {

		vec4 p;
		// invert the depth values since they represent the distance from the camera 
		// and OpenGL is looking down -z
		p.z = - depth_in.at<short>( in[i].pt.y, in[i].pt.x ) * 0.001f;

		if (p.z != 0)
		{
			// need to flip the coordinates to match the image coordinates later on
			p.x = -( in[ i ].pt.x / x_max - 0.5f ) * p.z * x_to_z;
			p.y = -( in[ i ].pt.y / y_max - 0.5f ) * p.z * y_to_z;
			p.w = 1;

			p = mvpw * p;

			if( p.x <= x_max && p.y <= y_max && p.x >= 0 && p.y >= 0 )
			{
				KeyPoint kp = in[ i ];
				kp.pt.x = roundf( p.x );
				kp.pt.y = roundf( p.y );
				kp_orth.push_back( kp );
				kp_out.push_back( in[i] );
			}
		}
	}

	in = kp_out;
	
	return kp_orth;
}

void openni_reprojector_gpu::reproject( const ci::gl::TextureRef &depth_in, const ci::gl::TextureRef &rgb_in )
{	
	//myUtilities::printGLErrors( "before" );

#ifdef PC_PERF_MARK
	span span(*marker_series_, L"start reproject GPU");
#endif // PC_PERF_MARK

	gl::ScopedGlslProg		shader_scp{ shader_ };
	gl::ScopedFramebuffer	fbScp{ fbo_ };
	gl::ScopedMatrices		matrices_scp{};
	gl::ScopedViewport		viewport_scp{ 0, 0, fbo_->getWidth(), fbo_->getHeight() };

	
	gl::clear( ColorA(0, 0, 0, 0));

	gl::setMatrices( cam_ );


	gl::setModelMatrix( mesh_model_mat_ );
	
	shader_->uniform( "depthKinect", 0 );
	shader_->uniform( "rgbKinect", 1 );
	shader_->uniform( "xMul_yMul_zMul_zAdd",
		vec4( x_to_z_, y_to_z_, transform_z_.mul, transform_z_.add ));
	shader_->uniform( "mesh_threshold", s_.mesh_threshold );
	
	gl::disableAlphaBlending();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableFaceCulling(false);

	gl::ScopedTextureBind	d_tex_Scp( depth_in, uint8_t( 0 ) );
	gl::ScopedTextureBind	rgb_tex_Scp( rgb_in, uint8_t( 1 ) );

	gl::draw( mesh_ );

	//myUtilities::printGLErrors( "after" );
	

	// fast pixel download with async PBO
	// https://www.opengl.org/discussion_boards/showthread.php/172509-Nvidia-Dual-Copy-Engines?s=fc5e57c81e27eabd1a32463295f1f650
	// https://vec.io/posts/faster-alternatives-to-glreadpixels-and-glteximage2d-in-opengl-es
	// http://www.songho.ca/opengl/gl_pbo.html#unpack

	
	
	gl::ScopedBuffer output_buff_scp( output_buff_ );
	glReadBuffer( GL_COLOR_ATTACHMENT0 );
	

	// normally one would use GL_BGRA since thats the OCV way
	// but we already feed a OCV img that was ordered wrong into the shader, so 
	// we use GL_RGBA to get the right colors
	// copy to PBO is async. getResult() should be called later 
	glReadPixels( 0, 0, fbo_->getWidth(), fbo_->getHeight(), GL_RGBA, GL_UNSIGNED_INT_8_8_8_8_REV, 0 );

	gl::ScopedBuffer copy_buff_scp( copy_buff_ );
	glCopyBufferSubData( GL_PIXEL_PACK_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, buffer_size_ );

	gl::enableAlphaBlending();
}

void openni_reprojector_gpu::depth_range_min( float val )
{
	s_.depth_min = val;
	transform_z_ = myUtilities::calcDataRangeTransformFactors( s_.depth_min, s_.depth_max, 0, 255 );
}

void openni_reprojector_gpu::depth_range_max( float val )
{
	s_.depth_max = val;
	transform_z_ = myUtilities::calcDataRangeTransformFactors( s_.depth_min, s_.depth_max, 0, 255 );
}

myUtilities::TransformFactors openni_reprojector_gpu::Transform_Z()
{
	return transform_z_;
}

void openni_reprojector_gpu::Mesh_rotation( ci::vec3 val )
{
	s_.mesh_rotation = val;
	mesh_rotation_mat_ = glm::rotate( glm::mat4( 1.0f ),  toRadians( val.x ), vec3( 1.f, 0.f, 0.f ) );
	mesh_rotation_mat_ = glm::rotate( mesh_rotation_mat_, toRadians( val.y ), vec3( 0.f, 1.f, 0.f ) );
	mesh_rotation_mat_ = glm::rotate( mesh_rotation_mat_, toRadians( val.z ), vec3( 0.f, 0.f, 1.f ) );
	mesh_model_mat_ = mesh_rotation_mat_ * mesh_translation_mat_;
}

void openni_reprojector_gpu::Mesh_translation( ci::vec3 val )
{
	s_.mesh_translation = val;
	mesh_translation_mat_ = glm::translate( glm::mat4( 1.0f ), val );
	mesh_model_mat_ = mesh_rotation_mat_ * mesh_translation_mat_;
}

void openni_reprojector_gpu::get_result( cv::Mat &rgb_out, cv::Mat &depth_out )
{
#ifdef PC_PERF_MARK
	span span( *marker_series_, L"get result reproject GPU" );
#endif // PC_PERF_MARK

	gl::ScopedBuffer copy_buff_scp( copy_buff_ );
	cv::Mat result( fbo_->getHeight(), fbo_->getWidth(), CV_8UC4 );

#ifdef PC_PERF_MARK
	marker_series_->write_flag( _T( "get start" ) );
#endif // PC_PERF_MARK

	// this blocks longer if called too early and the copy to cpu memory is not finished 
	glGetBufferSubData( GL_COPY_WRITE_BUFFER, 0, buffer_size_, result.data );
	
#ifdef PC_PERF_MARK
	marker_series_->write_flag( _T( "get stop" ) );
#endif // PC_PERF_M
	Mat out[] = { rgb_out, depth_out };

	int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };
	mixChannels( &result, 1, out, 2, from_to, 4 );

#ifdef RP_SHOW_DEBUG_IMG
	imshow( "Reprojector", result );
#endif // RP_SHOW_DEBUG_IMG
} 

void openni_reprojector_gpu::Camera_width( float w )
{
	s_.w_camera = w;
	float w_cam_half = s_.w_camera * 0.5f;
	float h_cam_half = w_cam_half * (s_.depth_h / (float)s_.depth_w);

	cam_.setOrtho(-w_cam_half, w_cam_half, -h_cam_half, h_cam_half, 1.f, 100.f);
}
