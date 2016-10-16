/*!
 * @file	person.h
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

#ifndef _H_PERSON
#define _H_PERSON

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "blob.h"
#include "hands.h"
#include "gesture_controller.h"
#include "shoot_gesture.h"

namespace pacher {

	class person;
	typedef std::shared_ptr<person> person_ptr;

	class person
	{
	public:

		person() = default;

		person( person&& other );

		explicit person( blob&& b );

		person& operator=( person&& other );

		void new_match( blob&& b, int matched_maxima );
		void update_gestures();
		
		static void reset_next_id();
		

		uint64_t id_ = 0;

		cv::Scalar color_ = cv::Scalar(255, 0, 0);

		boost::posix_time::ptime creation_time_;
		
		boost::posix_time::ptime disappeared_time_ ;

		blob blob_;

		// index of the local maxima that is chosen to be the head
		uint16_t head_index_ = 0;

		float head_height_avg_ = 0;

		bool matched_ = false;

		hands hands_;

		// TODO add setting that defines default gestures for newly created person objs
		std::vector<std::shared_ptr<gesture_controller>> gestures_ =	
			std::vector<std::shared_ptr<gesture_controller>> { std::make_shared<shoot_gesture>() };


	private:
		explicit person( blob&& b, uint64_t id );
		static uint64_t next_id_;
	};
}						// namespace pacher

#endif                  // include guard



