/*!
 * @file	mass_gameobj.h
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

#ifndef _H_MASS_GAMEOBJ
#define _H_MASS_GAMEOBJ

#include "suBox2D.h"
class masskerade_game;

class mass_gameobj
{
public:

	enum game_obj_type { PLAYER, PARTICLE, PROTON, NUM_OBJS};

	mass_gameobj() = default;
	mass_gameobj( mass_gameobj& other ) = delete;
	mass_gameobj& operator=( mass_gameobj& other ) = delete;

	game_obj_type get_type(){ return type_; }

	masskerade_game* get_game();

	void set_radius( float r );


	b2Body* physics_body_;
	ci::vec3 rotation_axis_ = ci::vec3( 0.f, 0.f, 1.f );

protected:
	virtual void draw();

	game_obj_type type_ = NUM_OBJS;
	masskerade_game* game_;

private:

};


#endif                  // include guard



