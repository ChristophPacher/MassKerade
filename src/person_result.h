/*!
 * @file	person_result.h
 * @author  Christoph Pacher <chris@christophpacher.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2015 Christoph Pacher http://www.christophpacher.com
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

#ifndef _H_PERSON_RESULT
#define _H_PERSON_RESULT

#include "person.h"
#include "hands.h"
#include "gesture_controller.h"

class person_result
{
public:
	person_result() = default;
	person_result( const pacher::person& p )
		:
		id_( p.id_ ),
		creation_time_( p.creation_time_ ),
		pos_(p.blob_.local_maxima_[p.head_index_].p),
		//contour_(p.blob_.contours_[0]),
		//bounding_b_(p.blob_.bounding_b_),
		hands_(p.hands_)
	{
		for (const auto &e : p.gestures_)
		{
			auto found = e->found();
			if ( found != gesture_controller::NOT_DETECTED )
			{
				found_gestures_.emplace_back( found );
			}
		}
	};


	uint64_t id_ = 0;

	boost::posix_time::ptime creation_time_;

	cv::Point2f pos_ = { 0.f, 0.f };

	//std::vector<cv::Point> contour_;

	//cv::Rect bounding_b_ = { 0, 0, 0, 0 };

	hands hands_;

	std::vector<gesture_controller::gesture_types> found_gestures_;

protected:

private:

};


#endif                  // include guard



