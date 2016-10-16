/*!
* @file	contact_listener.h
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

#ifndef _H_CONTACT_LISTENER
#define _H_CONTACT_LISTENER

#include <random>
#include <Box2D/Box2D.h>

class ContactListener : public b2ContactListener
{
public:
	ContactListener() = default;
	~ContactListener();
	ContactListener( ContactListener &other );
	ContactListener operator= (ContactListener &rhs);


	void listenToWorld( b2World *world );


	void BeginContact( b2Contact *contact ) override;


private:

	b2World     *mWorld = nullptr;

	std::random_device rdev_{};
	std::default_random_engine rnd_g_{ rdev_() };
	std::uniform_real_distribution<float> dist_m_p_one_{ -1.f, 1.f };
};
#endif                  // include guard