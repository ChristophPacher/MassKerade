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

#ifndef _H_OPENNI_REPROJECTOR_GPU
#define _H_OPENNI_REPROJECTOR_GPU

#include <ppl.h>
#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK

#include <opencv2/core/core.hpp>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/VboMesh.h>
#include <cinder/gl/BufferObj.h>

#include "pacher_utilities.h"

class openni_reprojector_gpu
{
public:
	struct settings 
	{
		float h_fov = 0.f;
		float v_fov = 0.f;
		float w_camera = 10.f;
		float depth_min = 2500.f;
		float depth_max = 5500.f;
		int depth_w = 640;
		int depth_h = 480;
		ci::vec3 mesh_rotation = ci::vec3();
		ci::vec3 mesh_translation = ci::vec3();
		ci::vec3 mesh_threshold = ci::vec3(0.01f, 0.01f, 0.01f);
	};

	openni_reprojector_gpu() = default;
	explicit openni_reprojector_gpu( const openni_reprojector_gpu::settings &s);

	void reproject( const ci::gl::TextureRef &depth_in, const ci::gl::TextureRef &rgb_in );
	std::vector<cv::KeyPoint> reproject( std::vector<cv::KeyPoint> &in, const cv::Mat &depth_in );

	void get_result( cv::Mat &rgb_out, cv::Mat &depth_out );

	void depth_range_min( float val );
	void depth_range_max( float val );
	void Camera_width( float w );

	ci::vec3 Mesh_rotation() const { return s_.mesh_rotation; }
	void Mesh_rotation( ci::vec3 val );
	void Mesh_translation( ci::vec3 val );
	myUtilities::TransformFactors Transform_Z();

	openni_reprojector_gpu::settings s_;
	ci::CameraOrtho			cam_;
private:
	
	float	x_to_z_, y_to_z_;

	myUtilities::TransformFactors	transform_z_;
	cinder::gl::FboRef	fbo_;
	ci::gl::GlslProgRef		shader_;
	ci::gl::VboMeshRef		mesh_;
	ci::gl::BufferObjRef output_buff_, copy_buff_;
	

	glm::mat4			mesh_rotation_mat_, mesh_translation_mat_, mesh_model_mat_, viewport_mat_;

	int buffer_size_;

#ifdef PC_PERF_MARK
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series_;
#endif // PC_PERF_MARK
};


#endif                  // include guard



