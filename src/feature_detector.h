/*!
 * @file	feature_detector.h
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

#ifndef _H_FEATURE_DETECTOR
#define _H_FEATURE_DETECTOR

#include <memory>
#include <ppl.h>
#ifdef PC_PERF_MARK
#include <cvmarkersobj.h>
#endif // PC_PERF_MARK


#include <opencv2\core\core.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>


class feature_detector
{
public:

	struct settings {
		int mask_erode_interations = 5;
		int detector_threshold = 10;
		float match_1_2_ratio = 0.8;
		float max_descriptor_distance_multi = 3;
		int max_keypoint_distance = 300;
	};

	feature_detector() = default;
	explicit feature_detector(const feature_detector::settings &s);

	std::vector<cv::KeyPoint> detect( const cv::Mat &rgb_in, const cv::Mat &mask = cv::Mat());

	cv::Mat describe( const cv::Mat &rgb_in, std::vector<cv::KeyPoint> &kp );

	std::vector<cv::DMatch> match( const cv::Mat &descr_old, const cv::Mat &descr_new,
		const std::vector<cv::KeyPoint> &kp_old, const std::vector<cv::KeyPoint> &kp_new );


	void detector_threshold( int threshold );
	
	
	feature_detector::settings settings_;

protected:
	cv::Ptr<cv::FastFeatureDetector> cv_detector_;
	cv::Ptr<cv::ORB> cv_extractor_ = cv::ORB::create();

	cv::BFMatcher cv_matcher_ = { cv_extractor_->defaultNorm() };


	
private:
	
	cv::Mat erode_kernel_ = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ) );
	int kps_min_ = ( std::numeric_limits<int>::max )( );
	int kps_max_ = 0;
	int matched_min_ = ( std::numeric_limits<int>::max )( );
	int matched_max_ = 0;

#ifdef PC_PERF_MARK
	std::shared_ptr<concurrency::diagnostic::marker_series> marker_series_;
#endif
};

#endif                  // include guard



