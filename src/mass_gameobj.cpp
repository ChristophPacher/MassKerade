/*!
 * @file	mass_gameobj.cpp
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

#include "mass_gameobj.h"

#include "masskerade_game.h"
#include "cinder/gl/gl.h"

using namespace ci;

void mass_gameobj::set_radius( float r )
{
	if ( physics_body_->GetFixtureList() )
	{
		b2CircleShape circle;
		circle.m_radius = r;
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &circle;

		fixtureDef.density = 1.0f;
		fixtureDef.friction = game_->s_.obj_friction;
		fixtureDef.restitution = game_->s_.obj_restitution;

		physics_body_->DestroyFixture( physics_body_->GetFixtureList() );
		physics_body_->CreateFixture( &fixtureDef );
	}
}

void mass_gameobj::draw()
{
	gl::ScopedModelMatrix model_scp{};
	b2Vec2 pos = physics_body_->GetWorldCenter();
	vec2 posc = game_->mScale.fromPhysics( vec2( pos.x, pos.y ) );
	gl::translate( posc.x, posc.y );
	gl::multModelMatrix( glm::rotate( physics_body_->GetAngle(), rotation_axis_ ) );

	// TODO make this a per object setting, if needed
	gl::scale( game_->s_.particle_radius, game_->s_.particle_radius, game_->s_.particle_radius );

	// TODO make this a pointer to the batch, so it can be different per child
	// if needed
	game_->batch_particle_->draw();
}

masskerade_game* mass_gameobj::get_game()
{
	return game_;
}