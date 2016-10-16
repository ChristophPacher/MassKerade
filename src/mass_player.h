/*!
 * @file	mass_player.h
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

#ifndef _H_MASS_PLAYER
#define _H_MASS_PLAYER

#include "mass_gameobj.h"

#include "person_result.h"


class mass_particle;
class mass_proton;

class mass_player : public mass_gameobj
{
public:

	struct settings{
		float player_radius = 10.f;
		float shot_speed = 7.f;
		float collected_speed = 3.f;
		float first_orbit = 0.5f;
		float orbit_incr = 0.2;
		ci::vec3 joint_Hz_Damp{ 3.f, 0.f, 0.f };
		std::vector<float> orbitals_map;
		std::vector<float> orbitals_map_distance;

		settings(){
			orbitals_fill_order = { 0, 1, 0, 2, 1, 0, 2 };
			particles_per_orbital = { 2, 4, 6 };
			calc_orbitals_map(); 
		
		};

		void calc_orbitals_map() {
			orbitals_map.clear();
			orbitals_map_distance.clear();
			for( const auto &orbit : orbitals_fill_order )
			{
				for( size_t particle = 0; particle < particles_per_orbital[ orbit ]; ++particle )
				{
					orbitals_map.push_back( orbit );
					orbitals_map_distance.push_back( first_orbit + orbit_incr * ( orbit ) );
				}
			};
		}

	private:
		std::vector<int> orbitals_fill_order;
		std::vector<int> particles_per_orbital;
	};

	// TODO make this not static and update all objs via game obj when setting changes
	static settings s_;

	mass_player()
	{
		type_ = mass_gameobj::PLAYER;
	};

	~mass_player();

	mass_player( const mass_player& other ) = delete;
	mass_player( mass_player&& other ) = delete;
	mass_player& mass_player::operator=( const mass_player& other ) = delete;
	mass_player& mass_player::operator=( mass_player&& other ) = delete;

	explicit mass_player( person_result&& pr, b2Body* physics_body, masskerade_game* game );


	void update( person_result&& pr, box2d::Scale& scale, float time_step, boost::posix_time::ptime now );
	void update_physics();

	virtual void draw() override;

	//void set_velocity( float x, float y ) {
	//	physics_body_->SetLinearVelocity( b2Vec2{ x, y } );
	//};

	void collect_particle( mass_particle* p );
	void splitoff_particle( size_t index, b2Vec2 direction, float speed, bool is_bullet );

	void reset_particle_orbits();

	void splitoff_particles( int num );
	void splitoff_later( mass_particle* p );
	
	void collect_proton( std::unique_ptr<mass_proton>&& p );
	void splitoff_proton_later( mass_proton* p );
	void debug_shot();

	person_result pr_;

	boost::posix_time::ptime last_seen_ = boost::posix_time::microsec_clock::universal_time();

	std::vector<mass_particle*> particles_;
	std::vector<mass_particle*> to_spilt_particles_;
	std::vector<b2DistanceJoint*> particle_joints_;

	std::vector<std::unique_ptr<mass_proton>> protons_;
	std::vector<mass_proton*> to_spilt_protons_;
	std::vector<b2DistanceJoint*> proton_joints_;


protected:

private:
	
};


#endif                  // include guard



