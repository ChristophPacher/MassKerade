/*!
 * @file	topdown_hand_tracker.h
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

#ifndef _H_TOPDOWN_HAND_TRACKER
#define _H_TOPDOWN_HAND_TRACKER

#include <mutex>

#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK

#include <opencv2/core/core.hpp>
#include <cinder/gl/Pbo.h>
#include <cinder/gl/Texture.h>

#include "person.h"
#include "person_result.h"

#include "background_substractor.h"
#include "openni_reprojector_gpu.h"
#include "blob_detector.h"
#include "feature_detector.h"
#include "local_maxima.h"
#include "topdown_hand_finder.h"


class topdown_hand_tracker
{
public:
	struct settings{
		float fov_h = 0;
		float fov_v = 0;
		int depthW = 0;
		int depthH = 0; 
		int rgbW = 0; 
		int rgbH = 0;
		int rgb_elem_size = 0;
		int depth_elem_size = 0;
		int disappeared_timeout_s = 10;
		int disappeared_match_radius_sq = 140*140;
		int new_person_min_area = 1000;
		int maxima_match_radius_sq = 40*40;
		int pos_match_radius_sq = 30 * 30;
		int area_match_max_diff = 1000;
		background_substractor::settings background_sub;
		openni_reprojector_gpu::settings reprojector;
		blob_detector::settings blob_detec;
		local_maxima::settings maxima;
		feature_detector::settings feature_detec;
		topdown_hand_finder::settings hand_finder;
	};

	struct maxima_match{
		maxima_match() = default;
		maxima_match( int index, float dist ) : maxima_index{ index }, distance{ dist } {};
		int maxima_index = -1;
		float distance = -1;
	};
	struct match_result{
		match_result() = default;
		match_result( int other, maxima_match m_match ) : other_index{ other }, maxima{ m_match } {};
		int other_index = - 1;
		maxima_match maxima = { -1, -1};
	};

	typedef std::vector<match_result> matches;
	typedef std::map<int, matches> blobs_matches;
	typedef std::vector<cv::KeyPoint> keypoints;

	topdown_hand_tracker() = default;
	explicit topdown_hand_tracker(topdown_hand_tracker::settings &s);

	std::vector<person_result> track(cv::Mat &depth_in, cv::Mat &rgb_in);

	void reset();

	cv::Mat draw_persons_info( bool ortho );
	cv::Mat draw_persons_info_depth( bool ortho );


	static std::mutex mutex_;


	topdown_hand_tracker::settings settings_;

	cv::Mat depth_backremoved_, color_backremoved_, depth_ortho_, color_ortho_, 
		color_ortho_masked_, contour_index_mask_, rgb_old_, color_info;

	background_substractor		back_sub_;
	openni_reprojector_gpu		reprojector_;
	blob_detector				blob_detector_;
	feature_detector			feature_detector_;
	local_maxima				maxima_;
	topdown_hand_finder			hand_finder_;

	ci::gl::PboRef rgb_pbo_, depth_backrem_pbo_;
	ci::gl::TextureRef	rgb_tex_, depth_backrem_tex_;

#ifdef PC_PERF_MARK
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series_;
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series1_;
#endif
	
private:
	void upload_rgb( cv::Mat &rgb_in );
	void upload_depth();
	int inside_blob( float y, float x );

	blobs_matches match( const std::vector<pacher::blob> &blobs, const std::vector<pacher::person> &persons, int match_radius );

	// returns index of maxima in blob that matched to head in person, no match when -1
	maxima_match match( const pacher::blob &b, const pacher::person &p, int match_radius );

	matches match( const pacher::blob &blob, const std::vector<pacher::person> &persons, int match_radius );

	std::vector<pacher::person> persons_, disappeared_;

	int rgb_buffer_size_ = 0;
	int depth_buffer_size_ = 0;


	
};


#endif                  // include guard



