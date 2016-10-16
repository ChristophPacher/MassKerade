/*!
 * @file	contact_filter.cpp
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

#include "contact_filter.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include "mass_particle.h"
#include "mass_player.h"
#include "mass_proton.h"

using namespace std;

contact_filter::~contact_filter()
{
	if (world_)
	{
		world_->SetContactListener(nullptr);
	}
}

contact_filter::contact_filter(contact_filter &other)
{
	set_world(other.world_);
	other.world_ = nullptr;
}

contact_filter contact_filter::operator=(contact_filter &rhs)
{
	contact_filter ret(rhs);
	return ret;
}

void contact_filter::set_world(b2World *world)
{
	if (world)
	{ 
		world->SetContactFilter(nullptr);
	}
	world_ = world;
	world_->SetContactFilter(this);
}


bool contact_filter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB)
{
	bool ret = true;
	if ( fixtureA->GetBody()->GetUserData() &&
		 fixtureB->GetBody()->GetUserData() )
	{
		// we want to make it easier to target other players by not 
		// allowing shots to collide with owner or other particles of the owner

		mass_gameobj* a = (mass_gameobj*)fixtureA->GetBody()->GetUserData();
		mass_gameobj* b = (mass_gameobj*)fixtureB->GetBody()->GetUserData();
		if( a->get_type() == mass_gameobj::PARTICLE ||
			b->get_type() == mass_gameobj::PARTICLE )
		{
			mass_particle* bullet = nullptr;
			mass_gameobj*  other = nullptr;

			// look if there is a bullet (if two bullets hit it doesn't matter which one we choose)
			if( a->get_type() == mass_gameobj::PARTICLE )
			{
				mass_particle* temp_particle = ( mass_particle* )a;
				if( temp_particle->is_bullet_ )
				{
					bullet = temp_particle;
					other = b;
				}
			}
			else
			{
				mass_particle* temp_particle = ( mass_particle* )b;
				if( temp_particle->is_bullet_ )
				{
					bullet = temp_particle;
					other = a;
				}
			}

			using namespace boost::posix_time;
			ptime now( microsec_clock::universal_time() );

			if ( bullet && now < bullet->collide_time_ )
			{
				
				if( other->get_type() == mass_gameobj::PLAYER )
				{
					mass_player* player = ( mass_player* ) other;

					if( bullet->owner_ == player || player->protons_.size() > 0 )
					{
						ret = false;
						cout << "did not collide with " << mass_gameobj::PLAYER << endl;
					}
				}
				else if( other->get_type() == mass_gameobj::PARTICLE )
				{
					mass_particle* other_particle = ( mass_particle* ) other;

					if( bullet->owner_ == other_particle->owner_ )
					{
						ret = false;
						cout << "did not collide with " << mass_gameobj::PARTICLE << endl;
					}
				}
				else if( other->get_type() == mass_gameobj::PROTON )
				{
					mass_proton* proton = ( mass_proton* ) other;

					if( bullet->owner_ == proton->owner_ )
					{
						ret = false;
						cout << "did not collide with " << mass_gameobj::PROTON << endl;
					}
				}
			}
		}
	}
	return ret;
}
