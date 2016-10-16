/*!
 * @file	mass_player.cpp
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

#include "mass_player.h"

#include "masskerade_game.h"
#include "mass_particle.h"
#include "mass_proton.h"
#include "gesture_controller.h"

using namespace std;

mass_player::~mass_player()
{
	//for( const auto &e : particle_joints_ )
	//{
	//		physics_body_->GetWorld()->DestroyJoint( e );
	//}
	while (particles_.size() > 0)
	{
		b2Vec2 dir = particles_[ 0 ]->physics_body_->GetWorldCenter()
			- physics_body_->GetWorldCenter();
		dir.Normalize();
		splitoff_particle( 0, dir, game_->get_rnd_split_speed(), false );
	}
	particles_.clear();
	particle_joints_.clear();
	game_->mSandbox.getWorld().DestroyBody(physics_body_);

	protons_.clear();
}

void mass_player::update_physics()
{

	// if not the same size then there were particles collected during the 
	// last box2d step. since we cannot create new joints during step
	// do it now
	if ( particle_joints_.size() != particles_.size() )
	{
		for ( size_t i = particle_joints_.size(), s = particles_.size(); i < s; ++i )
		{
			b2DistanceJointDef jd;
			jd.bodyA = physics_body_;
			jd.bodyB = particles_[ i ]->physics_body_;
			jd.frequencyHz = s_.joint_Hz_Damp.x;
			jd.dampingRatio = s_.joint_Hz_Damp.y;
			jd.collideConnected = false;
			jd.localAnchorA = physics_body_->GetLocalCenter();
			jd.localAnchorB = particles_[ i ]->physics_body_->GetLocalCenter();
			jd.length = s_.orbitals_map_distance[ particle_joints_.size() ];
			particles_[ i ]->orbit_ = s_.orbitals_map[ particle_joints_.size() ];

			particle_joints_.emplace_back(
				(b2DistanceJoint*)physics_body_->GetWorld()->CreateJoint( &jd )
				);

			particles_[ i ]->set_target_speed( s_.collected_speed );
		}
	}

	if ( proton_joints_.size() != protons_.size() )
	{
		for ( size_t i = proton_joints_.size(), s = protons_.size(); i < s; ++i )
		{
			b2DistanceJointDef jd;
			jd.bodyA = physics_body_;
			jd.bodyB = protons_[ i ]->physics_body_;
			jd.frequencyHz = game_->s_.proton_joint_Hz_Damp_len.x;
			jd.dampingRatio = game_->s_.proton_joint_Hz_Damp_len.y;
			jd.collideConnected = false;
			jd.localAnchorA = physics_body_->GetLocalCenter();
			jd.localAnchorB = protons_[ i ]->physics_body_->GetLocalCenter();
			jd.length = game_->s_.proton_joint_Hz_Damp_len.z;


			proton_joints_.emplace_back(
				(b2DistanceJoint*)physics_body_->GetWorld()->CreateJoint( &jd )
				);

			protons_[ i ]->set_target_speed( s_.collected_speed );
		}
	}

	if ( to_spilt_protons_.size() > 0 )
	{
		for ( size_t i = 0; i < protons_.size(); ++i )
		{
			for ( size_t j = 0; j < to_spilt_protons_.size(); ++j )
			{
				if ( protons_[ i ].get() == to_spilt_protons_[ j ] )
				{
					protons_.erase( protons_.begin() + i );
					to_spilt_protons_.erase( to_spilt_protons_.begin() + j );
				}
			}
		}
	}

	if ( protons_.size() == 0 && particles_.size() != 0 )
	{
		splitoff_particles( particles_.size() );
	}

	// check if we were hit during last step and we need to split off some
	// particles
	if ( to_spilt_particles_.size() > 0 )
	{
		for ( size_t i = 0; i < particles_.size();)
		{
			bool increment = true;
			for (size_t j = 0; j < to_spilt_particles_.size() && i < particles_.size();)
			{
				if ( particles_[ i ] == to_spilt_particles_[ j ] )
				{
					b2Vec2 dir = particles_[ i ]->physics_body_->GetWorldCenter()
						- physics_body_->GetWorldCenter();
					dir.Normalize();
					splitoff_particle( i, dir, game_->get_rnd_split_speed(), false );
					to_spilt_particles_.erase( to_spilt_particles_.begin() + j );
					increment = false;
					break;
				}
				else
				{
					++j;
				}
			}
			if (increment)
			{
				++i;
			}
		}
	}
}

void mass_player::update( person_result&& pr, box2d::Scale& scale, float time_step, boost::posix_time::ptime now )
{

	auto pos = scale.cam_to_physics( pr.pos_ );
	physics_body_->SetTransform( b2Vec2{ pos.x, pos.y }, 0.f );

	// this introduces jitter when one sets the pos and velocity
	//auto vel = scale.toPhysics( pr.pos_ - pr_.pos_ ) * ( 1.f / time_step );
	//physics_body_->SetLinearVelocity( b2Vec2{ vel.x, vel.y } );

	pr_ = move( pr );
	last_seen_ = now;
	

	for (const auto &e : pr_.found_gestures_)
	{
		// TODO pull gesture_types into a more accessible header
		if( e == gesture_controller::gesture_types::SHOOT && particles_.size() > 0)
		{
			size_t i_smallest = 0;
			float val_smallest = numeric_limits<float>::max();
			auto handpos = scale.cam_to_physics( pr_.hands_.h_[ 0 ].pos.p );
			b2Vec2 hand{ handpos.x, handpos.y };
			// only go through particles that have a joint setup already
			for( size_t i = 0, s = particle_joints_.size(); i < s; ++i )

			{ 
				
				float lenght = ( particles_[ i ]->physics_body_->GetWorldCenter() 
								 - hand ).LengthSquared();
				if (lenght < val_smallest )
				{
					val_smallest = lenght;
					i_smallest = i;
				}
			}
			b2Vec2 dir = hand - physics_body_->GetWorldCenter();
			dir.Normalize();
			particles_[ i_smallest ]->physics_body_->SetBullet( true );
			particles_[ i_smallest ]->physics_body_->SetTransform( hand, 0 );
			splitoff_particle( i_smallest, dir, s_.shot_speed, true );
		}
	}

}
void mass_player::splitoff_later( mass_particle* p )
{
	to_spilt_particles_.emplace_back( p );
}

void mass_player::splitoff_particles( int num )
{
	int counter = 0;
	for ( int i = particles_.size() - 1; i >= 0 && counter < num; --i, ++counter )
	{
		to_spilt_particles_.emplace_back( particles_[i] );
	}
}

void mass_player::splitoff_particle( size_t index, b2Vec2 direction, float speed, bool is_bullet )
{
	using namespace boost::posix_time;
	if (is_bullet)
	{
		particles_[ index ]->is_bullet_ = true;
		particles_[ index ]->collide_time_ = ptime( microsec_clock::universal_time() )
			+ seconds( 1 );
		auto contact = particles_[ index ]->physics_body_->GetContactList();
		
		// electrons are close to protons. box2d already creates a contact even though they 
		// are not touching.
		// they have to be destroyed because contact filter is not called otherwise 
		auto next = contact;
		while ( next )
		{
			contact = next;
			next = next->next;
			game_->mSandbox.getWorld().GetContactManager().Destroy( contact->contact );
		}
	}
	else
	{
		particles_[ index ]->is_bullet_ = false;
		particles_[ index ]->owner_ = nullptr;
		particles_[ index ]->collectable_time_ =  ptime( microsec_clock::universal_time() )
			+ seconds( game_->s_.split_uncollectable_s );
	}
	
	// the direction we wanna go 
	b2Vec2 impulse2 = direction;
	impulse2 *= speed * particles_[ index ]->physics_body_->GetMass();

	// cancel out the current velocity
	b2Vec2 impulse = -particles_[ index ]->physics_body_->GetLinearVelocity();
	impulse *= particles_[ index ]->physics_body_->GetMass();
	impulse += impulse2;

	particles_[ index ]->physics_body_->ApplyLinearImpulse( impulse,
		particles_[ index ]->physics_body_->GetWorldCenter(), true );

	particles_[ index ]->set_target_speed( speed );

	particles_[ index ]->set_angular_target_speed( game_->get_rnd_angular_speed() );

	particles_.erase( particles_.begin() + index );

	// it is possible that a player is deleted before the joint is created in update()
	// check if the index is actually in the vector before accessing
	if ( particle_joints_.size() > index )
	{
		physics_body_->GetWorld()->DestroyJoint( particle_joints_[ index ] );
		particle_joints_.erase( particle_joints_.begin() + index );
	}

	// we split off a particle in a possible random orbit, so we need to 
	// reset all particle orbits 
	reset_particle_orbits();

}

void mass_player::splitoff_proton_later( mass_proton* p )
{
	to_spilt_protons_.emplace_back( p );
}

void mass_player::collect_particle( mass_particle* p )
{
	if( particles_.size() < game_->s_.player_max_electrons && 
		particles_.size() < s_.orbitals_map.size() &&
		protons_.size() > 0 )
	{
		p->owner_ = this;
		particles_.emplace_back( p );
	}
	
}

void mass_player::collect_proton( std::unique_ptr<mass_proton>&& p )
{
	if ( protons_.size() < game_->s_.player_max_protons )
	{
		p->owner_ = this;
		protons_.emplace_back( move(p) );
	}
}

void mass_player::reset_particle_orbits()
{
	for ( size_t i = 0, s = particle_joints_.size(); i < s; ++i )
	{
		particle_joints_[ i ]->SetLength( s_.orbitals_map_distance[ i ] );
		particles_[ i ]->orbit_ = s_.orbitals_map[ i ];
	}
}

mass_player::settings mass_player::s_{};

mass_player::mass_player( person_result&& pr, b2Body* physics_body, masskerade_game* game ) 
	: pr_( std::move( pr ) )
{
	physics_body_ = physics_body;
	physics_body_->SetUserData(this);
	type_ = mass_gameobj::PLAYER;
	game_ = game;
}

void mass_player::draw()
{
	game_->shader_particle_->uniform( "colorA", game_->s_.colorA_shot );
	game_->shader_particle_->uniform( "colorB", game_->s_.colorB_shot );

	mass_gameobj::draw();
}

void mass_player::debug_shot()
{
	if (particle_joints_.size() > 0)
	{
		cout << "shot" << endl;
		b2Vec2 dir = physics_body_->GetWorldCenter() - particles_[ 0 ]->physics_body_->GetWorldCenter();
		dir.Normalize();
		particles_[ 0 ]->physics_body_->SetBullet( true );
		splitoff_particle( 0, dir, s_.shot_speed, true );
	}
}
