/*!
 * @file	masskerade_game.cpp
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

#include "masskerade_game.h"

#include <cinder/Cinder.h>
#include <cinder/gl/Context.h>
#include <cinder/app/AppNative.h>

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace boost::posix_time;

masskerade_game::masskerade_game( settings& s )
	: s_( s )
	, mScale{ s.w_tracker_input, s.h_tracker_input, s.off_screen_border }
	, dist_w_( 0, s_.w_tracker_input + 2 * s_.off_screen_border )
	, dist_h_( 0, s_.h_tracker_input + 2 * s_.off_screen_border )
	, dist_vel_( s_.particle_speed_min_max.x, s_.particle_speed_min_max.y  )
	, dist_vel_split( s_.particle_split_speed_min_max.x, s_.particle_split_speed_min_max.y )
	, dist_angular_( s_.particle_angular_speed_min_max.x, s_.particle_angular_speed_min_max.y )
{
	mSandbox.createBoundaryRect( mScale.toPhysics( 
		Rectf{ 0.f, 0.f, 
			   s_.w_tracker_input + s_.off_screen_border * 2 , 
			   s_.h_tracker_input + s_.off_screen_border * 2 } 
		) 
	);
	mSandbox.setGravity( { 0.f, 0.f } );
	mSandbox.setTimeStep( s_.fps );
	mContactListener.listenToWorld( &mSandbox.mWorld );
	contact_filter_.set_world( &mSandbox.mWorld );

	try {
		gl::GlslProg::Format format;
		format.vertex( loadAsset( "color_vert.glsl" ) )
			.fragment( loadAsset( "color_frag.glsl" ) );


		shader_particle_ = gl::GlslProg::create( format );
	}
	catch ( gl::GlslProgCompileExc exc ){
		cout << exc.what() << endl;
	}

	batch_particle_ = gl::Batch::create( geom::Icosahedron(), shader_particle_ );

	shader_particle_->uniform( "num_verts", (float)batch_particle_->getNumVertices() );

	float w_tracker_input_h = s_.w_tracker_input * 0.5f;
	float h_tracker_input_h = s_.h_tracker_input * 0.5f;
	camera_ = CameraOrtho( -w_tracker_input_h, w_tracker_input_h, 
						   -h_tracker_input_h, h_tracker_input_h,
						   1.f, 100.f );
	
	float w_center = w_tracker_input_h + s_.off_screen_border;
	float h_center = h_tracker_input_h + s_.off_screen_border;
	camera_.lookAt( vec3( w_center, h_center, 40.f ), vec3( w_center, h_center, 0.f ) );
}

void masskerade_game::update( std::vector<person_result> && prs )
{
	now_ = microsec_clock::universal_time();

		// go through the tracker results and create new players or update existing ones
	for ( auto &result : prs )
	{
		bool new_player = true;
		for (int i = 0, s = players_.size(); i < s ; ++i)
		{
			if( result.id_ == players_[ i ]->pr_.id_ )
			{
				players_[ i ]->update( move( result ), mScale, mSandbox.getTimeStep(), now_ );
				new_player = false;
				break;
			}
		}

		if (new_player)
		{
			add_player( move(result) );
		}
	}

	update();
}

void masskerade_game::update()
{
	now_ = microsec_clock::universal_time();

	if ( now_ > emit_particle_time_ && particles_.size() < s_.max_particles )
	{
		add_particle();
		emit_particle_time_ = now_ + seconds( dist_particle_time_(rnd_g_) );
	}

	for ( auto &p : players_)
	{
		p->update_physics();
	}

	// purge the vector of disappeared person objects that are too old
	players_.erase( remove_if( players_.begin(), players_.end(), [ &]( unique_ptr<mass_player> &p )
	{

		auto  dur = ( now_ - p->last_seen_ ).total_seconds();
		bool toDelete = dur > s_.disappeared_timeout_s;
		if ( toDelete )
		{
			cout << "player id " << p->pr_.id_ << " removed because no tracker update for " << dur << " seconds" << endl;
		}
		return toDelete;
	} ), players_.end() );


	// go through all the particles and accelerate or decelerate them 
	// when they are above or below their target speed
	float vel_min_sq = s_.particle_speed_min_max.x * s_.particle_speed_min_max.x;
	float vel_max_sq = s_.particle_speed_min_max.y * s_.particle_speed_min_max.y;
	for ( auto &p : particles_ )
	{
		b2Vec2 force{ 0.f, 0.f };
		if ( !p->is_bullet_ )
		{
		
			if ( p->owner_ )
			{
				// particles that circle around an owner need a force applied  
				// to keep circling 
				b2Vec2 r = p->physics_body_->GetWorldCenter() - p->owner_->physics_body_->GetWorldCenter();
				b2Vec2 tangent{ r.y, - r.x };
				b2Vec2 v = p->physics_body_->GetLinearVelocity();
			

				// orbits should rotate cw or ccw depending on even or odd orbit number
				if ( p->orbit_ % 2 == 0 )
				{
					tangent *= -1.f;
				}
				tangent.Normalize();

				float tangent_speed = b2Dot( tangent, v );


				// accelerate or decelerate, depending on collected_speed - tangent_speed
				force = tangent;
				force *= p->physics_body_->GetMass() * ( mass_player::s_.collected_speed - tangent_speed );
			
			}
			//else
			//{
			//	// some collisions can take or add energy even if restitution is 
			//	// set to 1, velocity changes.
			//	// we slow free particles down or speed them up to stay close to their target speed
			//	float speed_sq = p->physics_body_->GetLinearVelocity().LengthSquared();
			//	force = p->physics_body_->GetLinearVelocity();
			//	if ( p->target_speed_sq_ > speed_sq )
			//	{
			//		force *= 0.05f;
			//	}
			//	else
			//	{
			//		force *= -0.05f;
			//	}
			//}

			else
			{
				float speed_sq = p->physics_body_->GetLinearVelocity().LengthSquared();
				if ( speed_sq > vel_max_sq )
				{
					force = p->physics_body_->GetLinearVelocity();
					force *= -0.01f;
				}
				else if (speed_sq < vel_min_sq)
				{
					force = p->physics_body_->GetLinearVelocity();
					force *= 0.01f;
				}
			}
		}
		p->physics_body_->ApplyForceToCenter( force, true );
		// same with angular velocity
		//float torque = p->physics_body_->GetInertia() * ( p->target_angular_speed_ - p->physics_body_->GetAngularVelocity() );
		//p->physics_body_->ApplyTorque( torque, true );
	}

	mSandbox.step();
}

void masskerade_game::draw()
{
	gl::ScopedMatrices matrices_scp{};
	float scale = s_.h_window / s_.h_tracker_input;

	gl::ScopedViewport port_scp{ vec2{ ( s_.w_window - s_.w_tracker_input * scale ) / 2, 0 },
		vec2{ s_.w_tracker_input * scale, s_.h_window } };
	
	gl::setMatrices( camera_ );

	if ( draw_debug_ )
	{
		
		mSandbox.debugDraw( mScale.getPointsPerMeter() );
	}
	else
	{
		gl::ScopedAlphaBlend alpha_scp{ true };
		gl::ScopedFaceCulling culling_scp{ true };

		gl::ScopedGlslProg	shader_scp{ shader_particle_ };



		shader_particle_->uniform( "eye_pos_w", camera_.getEyePoint() );
		shader_particle_->uniform( "light_pos_w", vec4( s_.light_pos, s_.point_light ) );
		shader_particle_->uniform( "light_color", vec4( s_.light_color, s_.light_shinyness ) );
		shader_particle_->uniform( "ambient_color", s_.ambient_color );

		for( auto &e : particles_ )
		{
			e->draw();
		}


		for ( auto &player : players_ )
		{
			for ( auto &proton : player->protons_)
			{
				proton->draw();
			}
		}
		
	}

}

void masskerade_game::set_fps( float fps )
{
	s_.fps = fps;
	mSandbox.setTimeStep( 1.f/fps );
}

void masskerade_game::all_players_get_particle()
{
	for (const auto &p : players_)
	{
		add_particle();
		particles_.back().get()->physics_body_->SetTransform( p->physics_body_->GetWorldCenter(), 0 );
		p->collect_particle( particles_.back().get() );
	}
}

void masskerade_game::add_particle()
{
	int side = dist_sides_( rnd_g_ );

	vec2 pos;
	vec2 target{ dist_w_( rnd_g_ ), dist_h_( rnd_g_ ) };

	// top or bottom 
	if (side == 0 || side == 2)
	{
		pos.x = dist_w_( rnd_g_ );
		pos.y = s_.off_screen_border / 2;
		if( side == 2 )
			pos.y += s_.h_tracker_input;

		
	} 
	// left or right
	else
	{
		pos.y = dist_h_( rnd_g_ );
		pos.x = s_.off_screen_border / 2;
		if( side == 3 )
			pos.x += s_.w_tracker_input;

	}

	float target_vel = (float)dist_vel_( rnd_g_ );
	vec2 vel = glm::normalize( target - pos ) * target_vel;

	float angluar_vel = dist_angular_(rnd_g_);
	angluar_vel *= dist_binary_(rnd_g_) == 0 ? 1.f : -1.f;

	// TODO perhaps move this into particle ctor
	b2BodyDef b_def;
	b_def.type = b2_dynamicBody;
	b_def.linearDamping = s_.obj_damping;
	b_def.angularDamping = s_.obj_angular_damping;
	b_def.linearVelocity = b2Vec2(vel.x, vel.y);
	b_def.angularVelocity = angluar_vel;
	b_def.position.Set( mScale.toPhysics( pos.x ), mScale.toPhysics(pos.y) );


	b2CircleShape circle;
	circle.m_radius = mScale.toPhysics(s_.particle_radius);
	b2FixtureDef f_def;
	f_def.shape = &circle;
	f_def.density = 1.0f;
	f_def.friction = s_.obj_friction;
	f_def.restitution = s_.obj_restitution;

	b2Body *body = mSandbox.getWorld().CreateBody(&b_def);
	body->CreateFixture(&f_def);

	particles_.emplace_back(new mass_particle( 
		body,
		target_vel,
		vec3(dist_m_p_one_(rnd_g_), dist_m_p_one_(rnd_g_), dist_m_p_one_(rnd_g_)),
		angluar_vel,
		this));
}
void masskerade_game::add_player(person_result&& prs) 
{
	b2BodyDef b_def;
	b_def.type = b2_kinematicBody;
	b_def.linearDamping = s_.obj_damping;
	b_def.angularDamping = s_.obj_angular_damping;
	b_def.position = mScale.cam_to_physics(b2Vec2{ prs.pos_.x, prs.pos_.y });


	b2CircleShape circle;
	circle.m_radius = mScale.toPhysics(mass_player::s_.player_radius);
	b2FixtureDef f_def;
	f_def.shape = &circle;
	f_def.density = 1.0f;
	f_def.friction = s_.obj_friction;
	f_def.restitution = s_.obj_restitution;
	b2Body *body = mSandbox.getWorld().CreateBody(&b_def);
	body->CreateFixture(&f_def);

	// TODO move ownership (unique ptr) of electrons and 
	// only free electrons in game_.particles
	players_.emplace_back(new mass_player( move(prs), body, this));



	b_def.type = b2_dynamicBody;
	circle.m_radius = mScale.toPhysics( s_.particle_radius );
	f_def.shape = &circle;
	b2Vec2 pos = players_.back()->physics_body_->GetWorldCenter();
	b_def.linearVelocity = b2Vec2{ 0, 0 };
	b_def.linearDamping = s_.proton_damp_dampang_restitution.x;
	b_def.angularDamping = s_.proton_damp_dampang_restitution.y;
	
	f_def.friction = 0.5;
	f_def.restitution = s_.proton_damp_dampang_restitution.z;

	for (size_t i = 0; i < s_.player_new_protons; ++i)
	{
		
		b_def.position.Set( pos.x + dist_m_p_one_( rnd_g_ ) / 2.f, pos.y + dist_m_p_one_( rnd_g_ ) / 2.f );
		body = mSandbox.getWorld().CreateBody( &b_def );
		body->CreateFixture( &f_def );


		players_.back()->collect_proton( unique_ptr<mass_proton>( new mass_proton(
			body,
			s_.proton_speed,
			vec3( dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ) ),
			get_rnd_angular_speed(),
			players_.back().get(),
			this ) ) );
	}
}

void masskerade_game::reset()
{
	for ( auto &e : players_)
	{
		e->particle_joints_.clear();
		e->particles_.clear();
		e->to_spilt_particles_.clear();
	}
	particles_.clear();

	protons_.clear();
	players_.clear();
}

void masskerade_game::set_player_first_orbit( float d )
{
	mass_player::s_.first_orbit = d;
	update_player_orbits();
}

void masskerade_game::set_player_orbit_incr( float d )
{
	mass_player::s_.orbit_incr = d;
	update_player_orbits();
}

void masskerade_game::update_player_orbits()
{
	mass_player::s_.calc_orbitals_map();
	for (const auto &e : players_)
	{
		e->reset_particle_orbits();
	}
}
void masskerade_game::set_particle_collected_vel_( float vel )
{
	mass_player::s_.collected_speed = vel;
	for ( auto &p : particles_ )
	{
		if ( !p->is_bullet_ && p->owner_ )
		{
			p->set_target_speed( mass_player::s_.collected_speed );
		}
	}
}

void masskerade_game::set_particle_vel_( ci::vec2 min_max )
{
	s_.particle_speed_min_max = min_max;
	dist_vel_ = uniform_real_distribution<float>{ min_max.x, min_max.y };
	for ( auto &p : particles_ )
	{
		if ( !p->is_bullet_ && !p->owner_ )
		{
			p->set_target_speed( get_rnd_speed() );
		}
	}
}

void masskerade_game::set_particle_split_vel_( ci::vec2 min_max )
{
	s_.particle_split_speed_min_max = min_max;
	dist_vel_split = uniform_real_distribution < float > { min_max.x, min_max.y };
}

void masskerade_game::set_particle_angluar_vel_( ci::vec2 min_max )
{
	s_.particle_angular_speed_min_max = min_max;
	dist_angular_ = uniform_real_distribution<float>{ min_max.x, min_max.y };
	for ( auto &p : particles_ )
	{
		p->target_angular_speed_ = get_rnd_angular_speed();
	}
} 
void masskerade_game::set_player_radius( float r)
{
	mass_player::s_.player_radius = r;
	for ( auto &e : players_ )
	{
		e->set_radius( mScale.toPhysics( r ));
	}
}
void masskerade_game::set_player_orbit_joint( ci::vec3 settings )
{
	mass_player::s_.joint_Hz_Damp = settings;
	for ( auto &p : players_ )
	{
		for ( auto &j : p->particle_joints_)
		{
			j->SetFrequency( settings.x );
			j->SetDampingRatio( settings.y );
		}
	}
}

void masskerade_game::set_player_proton_joint( ci::vec3 settings )
{
	s_.proton_joint_Hz_Damp_len = settings;
	for ( auto &p : players_ )
	{
		for ( auto &j : p->proton_joints_ )
		{
			j->SetFrequency( settings.x );
			j->SetDampingRatio( settings.y );
			j->SetLength( settings.z );
		}
	}
}

void masskerade_game::set_proton_damp_dampang_restituion( ci::vec3 settings )
{
	s_.proton_damp_dampang_restitution = settings;
	for ( auto &p : players_ )
	{
		for ( auto &pro : p->protons_ )
		{
			pro->set_damp_dampang_restituion( settings );
		}
	}

	for ( auto &pro : protons_ )
	{
		pro->set_damp_dampang_restituion( settings );
	}
}

void masskerade_game::set_particle_radius( float r )
{
	s_.particle_radius = r;
	for ( auto &e : particles_ )
	{
		e->set_radius( mScale.toPhysics( r ) );
	}
}

float masskerade_game::get_rnd_speed()
{
	return dist_vel_( rnd_g_ );
}

float masskerade_game::get_rnd_split_speed()
{
	return dist_vel_split( rnd_g_ );
}

float masskerade_game::get_rnd_angular_speed()
{
	return dist_angular_( rnd_g_ );
}

void masskerade_game::player_shoot_particle()
{
	if (players_.size() > 0 )
	{
		players_[ 0 ]->debug_shot();
	}
}
