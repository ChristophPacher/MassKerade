/*!
 * @file	mass_particle.h
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

#ifndef _H_MASS_PARTICLE
#define _H_MASS_PARTICLE

#include "mass_gameobj.h"

#include <boost/date_time/posix_time/posix_time.hpp>

class mass_player;

class mass_particle : public mass_gameobj
{
public:
	mass_particle();
	~mass_particle();

	explicit mass_particle( b2Body* physics_body, 
							float target_speed, 
							ci::vec3 rotation_axis, 
							float target_angular_speed,
							masskerade_game* game);

	mass_particle( mass_particle& other ) = delete;
	mass_particle& operator=( mass_particle& other ) = delete;

	virtual void draw() override;

	void set_target_speed( float speed );
	void set_angular_target_speed( float s );
	void shot_hit( bool hit_other_player_obj = false);


	mass_player* owner_ = nullptr;
	bool is_bullet_ = false;

	int orbit_ = 0;
	int bounces_ = 0;

	float target_speed_ = 0.f;
	float target_speed_sq_ = 0.f;
	float target_angular_speed_ = 0.f;
	

	uint64_t id_ = -1;

	boost::posix_time::ptime collectable_time_{ boost::posix_time::microsec_clock::universal_time() };
	boost::posix_time::ptime collide_time_{ boost::posix_time::microsec_clock::universal_time() };
	
protected:

private:
	static uint64_t next_id_;
};

#endif                  // include guard



