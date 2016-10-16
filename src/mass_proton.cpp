/*!
 * @file	mass_proton.cpp
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

#include "mass_proton.h"

#include "masskerade_game.h"

using namespace std;

uint64_t mass_proton::next_id_ = 0;

mass_proton::mass_proton( b2Body* physics_body, 
							  float target_speed, 
							  ci::vec3 rotation_axis, 
							  float target_angular_speed,
							  mass_player* owner,
							  masskerade_game* game )
	: target_speed_			( target_speed )
	, target_speed_sq_		( target_speed * target_speed )

	, target_angular_speed_	( target_angular_speed )
	, owner_				( owner )
{
	physics_body_ = physics_body;
	physics_body_->SetUserData(this);
	rotation_axis_ = rotation_axis;
	type_ = mass_gameobj::PROTON;
	id_ = ++next_id_;
	game_ = game;
}

mass_proton::mass_proton()
{
	type_ = mass_gameobj::PROTON;
}

void mass_proton::set_target_speed( float speed )
{
	target_speed_ = speed;
	target_speed_sq_ = speed * speed;
}

void mass_proton::set_angular_target_speed( float s )
{
	target_angular_speed_ = s;
}

mass_proton::~mass_proton()
{
	// TODO we only need this for game.remove_all_particles() 
	// if we ever want to selectively destroy a proton 
	// check owner and clear particle there, before destruction
	game_->mSandbox.getWorld().DestroyBody( physics_body_ );
}

void mass_proton::draw()
{
	game_->shader_particle_->uniform( "colorA", game_->s_.colorA_proton );
	game_->shader_particle_->uniform( "colorB", game_->s_.colorB_proton );
	mass_gameobj::draw();
}

void mass_proton::set_damp_dampang_restituion( ci::vec3 settings )
{
	physics_body_->SetLinearDamping( settings.x );
	physics_body_->SetAngularDamping( settings.y );
	physics_body_->GetFixtureList()->SetRestitution( settings.z );
}
