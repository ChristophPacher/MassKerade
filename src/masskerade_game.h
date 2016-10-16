/*!
 * @file	masskerade_game.h
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

#ifndef _H_MASSKERADE_GAME
#define _H_MASSKERADE_GAME

#include <random>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <cinder/gl/GlslProg.h>
#include <cinder/gl/Batch.h>

#include "suBox2D.h"

#include "person_result.h"
#include "mass_player.h"
#include "mass_particle.h"
#include "mass_proton.h"

#include "ContactListener.h"
#include "contact_filter.h"


class masskerade_game
{
	friend class mass_gameobj;
	friend class mass_player;
	friend class mass_particle;
	friend class mass_proton;

public:
	typedef std::unique_ptr<masskerade_game> UPtr;

	struct settings {
		float fps = 30.f;
		float w_tracker_input = 0.f;
		float h_tracker_input = 0.f;
		float off_screen_border = 100.f;
		float w_window = 0.f;
		float h_window = 0.f;
		float camera_fov_v = 0.f;
		float disappeared_timeout_s = 2.f;
		int max_particles = 20;
		ci::vec3 light_pos{};
		float point_light = 1.f;
		float light_shinyness = 80.f;
		ci::Color colorA{ 1.f, 0.f, 0.f };
		ci::Color colorB{ 0.f, 1.f, 0.f };
		ci::Color colorA_shot{ 1.f, 1.f, 0.f };
		ci::Color colorB_shot{ 0.f, 1.f, 1.f };
		ci::Color colorA_collected{ 0.f, 0.f, 1.f };
		ci::Color colorB_collected{ 0.f, 1.f, 1.f };
		ci::Color colorA_splitoff{ 1.f, 1.f, 1.f };
		ci::Color colorB_splitoff{ 0.8f, 0.8f, 0.8f };
		ci::Color ambient_color{ 0.1f, 0.1f, 0.1f };
		ci::vec3 light_color{ 1.f, 1.f, 1.f };
		float particle_radius = 10.f;
		ci::vec2 particle_speed_min_max{ 1.f, 5.f };
		ci::vec2 particle_split_speed_min_max{ 3.f, 5.f };
		ci::vec2 particle_angular_speed_min_max{ 0.5f, 2.f };
		float obj_restitution = 1.f;
		float obj_friction = 0.f;
		float obj_angular_friction = 0.f;
		float obj_damping = 0.f;
		float obj_angular_damping = 0.f;
		float max_shot_bounces = 3.f;
		float split_uncollectable_s = 2.f;
		int player_new_protons = 3;
		int player_max_protons = 5;
		int player_max_electrons = 20;
		float proton_speed = 0.0f;
		ci::vec3 split_off_stages{ 5, 10, 15 };
		ci::vec3 proton_joint_Hz_Damp_len{ 3.f, 0.f, 0.1f };
		ci::vec3 proton_damp_dampang_restitution{ 0.5f, 0.5f, 0.5f };
		ci::Color colorA_proton{ 1.f, 1.f, 0.f };
		ci::Color colorB_proton{ 0.f, 1.f, 1.f };
	};

	masskerade_game()= default;
	explicit masskerade_game( settings& s );

	void update( std::vector<person_result> && prs  );
	void update();
	void draw();

	void add_particle();
	void all_players_get_particle();
	void reset();

	void set_fps( float fps );

	void set_player_first_orbit( float d );
	void set_player_orbit_incr( float d );
	void set_player_radius( float r );
	void set_player_orbit_joint( ci::vec3 settings );
	void set_player_proton_joint( ci::vec3 settings );

	void set_particle_radius( float r );

	void toogle_draw_debug() { draw_debug_ = !draw_debug_; }

	float get_rnd_speed();
	float get_rnd_angular_speed();
	float get_rnd_split_speed();

	void set_particle_vel_( ci::vec2 min_max );
	void set_particle_angluar_vel_( ci::vec2 min_max );
	void set_particle_collected_vel_( float vel );
	void set_particle_split_vel_( ci::vec2 min_max );

	void set_proton_damp_dampang_restituion( ci::vec3 settings );

	void player_shoot_particle();

	settings s_;
protected:
	b2::Sandbox         mSandbox;
	b2::Scale           mScale;
	boost::posix_time::ptime now_{ boost::posix_time::microsec_clock::universal_time() };
private:
	void add_player( person_result&& prs );
	void update_player_orbits();
	

	
	ContactListener   mContactListener;
	contact_filter contact_filter_;


	
	std::vector<std::unique_ptr<mass_particle>> particles_;
	std::vector<std::unique_ptr<mass_proton>> protons_;
	std::vector<std::unique_ptr<mass_player>> players_;

	std::random_device rdev_{};
	std::default_random_engine rnd_g_{rdev_()};
	std::uniform_int_distribution<int> dist_sides_{ 0, 3 };
	std::uniform_int_distribution<int> dist_w_{ 0, 640 };
	std::uniform_int_distribution<int> dist_h_{ 0, 480 };
	std::uniform_real_distribution<float> dist_vel_{ 1, 5 };
	std::uniform_real_distribution<float> dist_vel_split{ 3, 5 };
	std::uniform_real_distribution<float> dist_angular_{ 0.5f, 2.f };
	std::uniform_int_distribution<int> dist_binary_{ 0, 1 };
	std::uniform_real_distribution<float> dist_m_p_one_{ -1.f, 1.f };
	std::uniform_int_distribution<int> dist_particle_time_{ 2, 7 };

	boost::posix_time::ptime emit_particle_time_{ boost::posix_time::microsec_clock::universal_time() };

	ci::CameraOrtho			camera_;
	ci::gl::GlslProgRef		shader_particle_;
	ci::gl::BatchRef		batch_particle_;

	bool draw_debug_ = false;

};


#endif                  // include guard



