/*!
 * @file	person.cpp
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

#include "person.h"
#include "local_maxima.h"

using namespace pacher;
using namespace boost::posix_time;
using namespace std;

uint64_t person::next_id_ = 0;


person::person( blob&& b )
	: id_( ++person::next_id_ )
	, creation_time_( microsec_clock::universal_time() )
	, color_( sin( 0.8 * id_ ) * 127 + 128,
			sin( 0.8 * id_ + 2 ) * 127 + 128,
			sin( 0.8 * id_ + 4 ) * 127 + 128 )
	, matched_( true )
	, blob_( move( b ) )

{
	
	if ( blob_.local_maxima_.size() > 1 )
	{
		// when there is more than one local max we chose the highest to be the head
		for ( int i = 0, l = blob_.local_maxima_.size(); i < l; ++i )
		{
			if ( blob_.local_maxima_[ i ].v > blob_.local_maxima_[ head_index_ ].v )
			{
				head_index_ = i;
			}
		}
	}


	if ( blob_.local_maxima_[ head_index_ ].v > 0 )
	{
		head_height_avg_ = blob_.local_maxima_[ head_index_ ].v;
	}
}

pacher::person::person( person&& other )
	: id_( other.id_ )
	, color_( other.color_ )
	, creation_time_( other.creation_time_ )
	, disappeared_time_( other.disappeared_time_ )
	, blob_( std::move( other.blob_ ) )
	, head_index_( other.head_index_ )
	, head_height_avg_( other.head_height_avg_ )
	, matched_( other.matched_ )
	, hands_( other.hands_ )
	, gestures_( std::move( other.gestures_ ) )
{
	other.id_ = 0;
}

person& pacher::person::operator=( person&& other )
{
	id_ = other.id_;
	color_ = other.color_;
	creation_time_ = other.creation_time_;
	disappeared_time_ = other.disappeared_time_;
	blob_ = std::move( other.blob_ );
	head_index_ = other.head_index_;
	head_height_avg_ = other.head_height_avg_;
	matched_ = other.matched_;
	hands_ = other.hands_;
	gestures_ = std::move( other.gestures_ );

	other.id_ = 0;
	return *this;
}

void person::new_match( blob&& b, int matched_maxima )
{
	blob_ = move( b );
	head_index_ = matched_maxima;
	matched_ = true;
	float speed = 0.002f;
	head_height_avg_ = head_height_avg_ * ( 1 - speed ) + blob_.local_maxima_[ head_index_ ].v * speed;
}

void person::reset_next_id()
{
	next_id_ = 0;
}

void pacher::person::update_gestures()
{
	for (const auto &gc : gestures_)
	{
		gc->update( hands_ );
	}
}
