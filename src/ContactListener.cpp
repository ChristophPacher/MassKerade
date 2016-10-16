/*!
* @file	ContactListener.cpp
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

#include "ContactListener.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <cinder/Vector.h>

#include "mass_particle.h"
#include "mass_player.h"
#include "mass_proton.h"
#include "masskerade_game.h"

using namespace std;

ContactListener::~ContactListener()
{
  if( mWorld )
  {
    mWorld->SetContactListener( nullptr );
  }
}

ContactListener::ContactListener( ContactListener &other )
{
  listenToWorld( other.mWorld );
  other.mWorld = nullptr;
}

ContactListener ContactListener::operator=( ContactListener &rhs )
{
  ContactListener ret( rhs );
  return ret;
}

void ContactListener::listenToWorld( b2World *world )
{
  if( world )
  { // unregister previous world
    world->SetContactListener( nullptr );
  }
  mWorld = world;
  mWorld->SetContactListener( this );
}

void ContactListener::BeginContact( b2Contact *contact )
{

	if( contact->GetFixtureA()->GetBody()->GetUserData() &&
		contact->GetFixtureB()->GetBody()->GetUserData() )
	{
		mass_gameobj* a = ( mass_gameobj* ) contact->GetFixtureA()->GetBody()->GetUserData();
		mass_gameobj* b = ( mass_gameobj* ) contact->GetFixtureB()->GetBody()->GetUserData();

		if( a->get_type() == mass_gameobj::PARTICLE && b->get_type() == mass_gameobj::PARTICLE )
		{
			mass_particle* a_particle = ( mass_particle* )a;
			mass_particle* b_particle = ( mass_particle* )b;

			if ( a_particle->is_bullet_ && b_particle->is_bullet_ )
			{
				// two bullets hit each other
				a_particle->shot_hit();
				b_particle->shot_hit();
			}
			else if ( a_particle->is_bullet_ || b_particle->is_bullet_ )
			{
				// a bullet hit another particle
				mass_particle* shot = nullptr;
				mass_particle* particle = nullptr;

				if (a_particle->is_bullet_)
				{
					shot = a_particle;
					particle = b_particle;
				}
				else
				{
					shot = b_particle;
					particle = a_particle;
				}


				if (particle->owner_ && particle->owner_ != shot->owner_ )
				{	
					// The bullet hit a particle that is owned by someone else 
					// so we split it off its owner
					//particle->owner_->splitoff_later( particle );
					int num_to_split = 1;
					if ( particle->owner_->particles_.size() > (int)particle->get_game()->s_.split_off_stages.z )
					{
						num_to_split = 4;
					}
					if ( particle->owner_->particles_.size() > (int)particle->get_game()->s_.split_off_stages.y )
					{
						num_to_split = 3;
					}
					else if ( particle->owner_->particles_.size() > (int)particle->get_game()->s_.split_off_stages.x )
					{
						num_to_split = 2;
					}
					particle->owner_->splitoff_particles( num_to_split );

					shot->shot_hit( true );
				}
				else
				{
					// we hit a free particle
					shot->shot_hit();
				}
			}

			// change the rotation axis unscientifically to a random new one 
			a_particle->rotation_axis_ = ci::vec3( dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ) );
			b_particle->rotation_axis_ = ci::vec3( dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ), dist_m_p_one_( rnd_g_ ) );
			
		}
		else if( ( a->get_type() == mass_gameobj::PARTICLE && b->get_type() == mass_gameobj::PLAYER) || 
			( a->get_type() == mass_gameobj::PLAYER && b->get_type() == mass_gameobj::PARTICLE ) )
		{
			mass_player* player = nullptr;
			mass_particle* particle = nullptr;

			if( a->get_type() == mass_gameobj::PARTICLE )
			{
				particle = (mass_particle*) a;
				player	 = (mass_player*)   b;
			}
			else
			{
				particle = (mass_particle*) b;
				player	 = (mass_player* )  a;
			}
			using namespace boost::posix_time;
			ptime now( microsec_clock::universal_time() );
			
			if ( particle->is_bullet_ )
			{
				//if ( particle->owner_->pr_.id_ != player->pr_.id_ )
				//{
				//	// The particle is a bullet and hit another player so we split off 
				//	// several particles 
				//	player->splitoff_particles( 3 );

				//	particle->shot_hit( true );
				//} 
				//else
				//{
				//	// shot hit the owner
				//	particle->shot_hit();
				//}
			}
			else if( !particle->owner_ && now > particle->collectable_time_ )
			{
				// a free particle hit a player, collect it
				player->collect_particle( particle );
			}
		}
		else if ( ( a->get_type() == mass_gameobj::PARTICLE && b->get_type() == mass_gameobj::PROTON ) ||
				  ( a->get_type() == mass_gameobj::PROTON && b->get_type() == mass_gameobj::PARTICLE ) )
		{
			mass_proton* proton = nullptr;
			mass_particle* particle = nullptr;

			if ( a->get_type() == mass_gameobj::PARTICLE )
			{
				particle = (mass_particle*)a;
				proton = (mass_proton*)b;
			}
			else
			{
				particle = (mass_particle*)b;
				proton = (mass_proton*)a;
			}

			if ( particle->is_bullet_ )
			{
				if ( particle->owner_->pr_.id_ != proton->owner_->pr_.id_ )
				{
					// The particle is a bullet and hit a proton of another player
					proton->owner_->splitoff_proton_later( proton );

					particle->shot_hit( true );
				}
				else
				{
					// shot hit the owner
					particle->shot_hit();
				}
			}
		}
	} 
	else 
	{
		mass_gameobj* game_obj = nullptr;
		if( contact->GetFixtureA()->GetBody()->GetUserData() )
			game_obj = ( mass_gameobj* )contact->GetFixtureA()->GetBody()->GetUserData();
		else 
		if( contact->GetFixtureB()->GetBody()->GetUserData() )
			game_obj = ( mass_gameobj* )contact->GetFixtureB()->GetBody()->GetUserData();

		if ( game_obj && game_obj->get_type() == mass_gameobj::PARTICLE )
		{
			mass_particle* particle = ( mass_particle* )game_obj;
			
			// the particle hit a wall since the walls have no userdata
			if (particle->owner_ && particle->is_bullet_)
			{
				particle->shot_hit();
			}
		}
	}
}
