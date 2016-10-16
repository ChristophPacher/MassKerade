/*!
 * @file	blob.h
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

#ifndef _H_BLOB
#define _H_BLOB

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/geometry.hpp>

#include "boost_geometry_opencv_register.h"
#include "local_maxima.h"




namespace pacher {

	typedef std::vector<cv::Point> contour;

	class blob
	{
	public:

		blob() = default;

		blob( blob&& other )
			: mask_(other.mask_)
			, area_(other.area_)
			, contours_(std::move(other.contours_))
			, bounding_b_(other.bounding_b_)
			, pos_(other.pos_)
			, local_maxima_(std::move(other.local_maxima_))
			, kps_(std::move(other.kps_))
			, kps_ortho_( std::move(other.kps_ortho_))
		{};

		blob& operator=( blob&& other )
		{
			mask_ = other.mask_;
			area_ = other.area_;
			contours_ = std::move( other.contours_ );
			bounding_b_ = other.bounding_b_;
			pos_ = other.pos_;
			local_maxima_ = std::move( other.local_maxima_ );
			kps_ = std::move( other.kps_ );
			kps_ortho_ = std::move( other.kps_ortho_ );
			return *this;
		};

		explicit blob( contour&& c, int mask_w, int mask_h )
		{
			init( move( c ), mask_h, mask_w );
			area_ = boost::geometry::area( contours_[ 0 ] );
		};


		explicit blob( contour&& c, int mask_w, int mask_h, float area )
			:
			area_			( area )
		{
			init( move( c ), mask_h, mask_w );
		};

		explicit blob( contour&& c, int mask_w, int mask_h, 
			  std::vector<cv::KeyPoint>&& kps, 
			  std::vector<cv::KeyPoint>&& kps_ortho, 
			  local_maxima::PointValue h )
			:
			kps_( std::move( kps ) ),
			kps_ortho_( std::move( kps_ortho ) ),
			bounding_b_( cv::boundingRect( c ) )
		{
			init( move( c ), mask_h, mask_w );

			area_ = boost::geometry::area( contours_[0] );
			local_maxima_.push_back( h );
		};



		// a mask that marks the pixels inside the camera frame that belong to this blob
		cv::Mat mask_;

		float area_ = -1;

		// contours
		std::vector<contour> contours_;

		// the blobs axis aligned bounding box
		cv::Rect bounding_b_ = { 0, 0, 0, 0};

		cv::Point2f pos_ = {0.f, 0.f};

		std::vector<local_maxima::PointValue> local_maxima_;

		std::vector<cv::KeyPoint> kps_;
		std::vector<cv::KeyPoint> kps_ortho_;

	private:
		void init( contour&& c, int mask_h, int mask_w ){
			bounding_b_ = cv::boundingRect( c );
			contours_.push_back( std::move( c ) ),
				mask_ = cv::Mat( mask_h, mask_w, CV_8UC1, cv::Scalar( 0 ) );
			cv::drawContours( mask_, contours_, 0, cv::Scalar( 1 ), cv::FILLED );
			boost::geometry::correct( contours_[ 0 ] );
			boost::geometry::centroid( contours_[ 0 ], pos_ );
		};

	};

	typedef std::vector<blob> blobs;

}						// namespace pacher


#endif                  // include guard



