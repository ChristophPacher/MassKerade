/*!
 * @file	shoot_gesture.cpp
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

#include "shoot_gesture.h"


shoot_gesture::settings shoot_gesture::s_{};

void shoot_gesture::update( const hands &hs )
{
	found_ = false;
	const boost::posix_time::ptime now( boost::posix_time::microsec_clock::universal_time() );
	if( !hands_were_straight_ && hs.h_[ 0 ].extended && hs.h_[ 1 ].extended && hs.angle_ > s_.hands_straight_angle_rad )
	{
		// hands are straight, remember when
		hands_were_straight_ = true;
		hands_straight_time_ = now;
	}
	else if( hands_were_straight_ && hs.touching_ &&
		s_.extend_to_shoot_delay_ms < ( now - hands_straight_time_ ).total_milliseconds() )
	{
		// hands were straight and are touching now insight our time frame
		// this is a shot
		found_ = true;
		hands_were_straight_ = false;
	}
	else if ( s_.extend_to_shoot_max_delay_ms < ( now - hands_straight_time_ ).total_milliseconds() )
	{
		// if too much time passed since hands were straight, for get about it
		hands_were_straight_ = false;
	}
}
