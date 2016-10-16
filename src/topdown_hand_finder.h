/*!
 * @file	topdown_hand_finder.h
 * @author  Christoph Pacher <chris@christophpacher.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2014 Christoph Pacher http://www.christophpacher.com
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

#ifndef _H_TOPDOWN_HAND_FINDER
#define _H_TOPDOWN_HAND_FINDER

#include "person.h"

#include <tuple>

class topdown_hand_finder
{
public:
	struct settings
	{
		float height_to_headradius = 0.2f;
		float height_to_handthreshold = 0.45f;
		double hands_min_size = 50;
		int defect_min_depth = 1000;
	};

	topdown_hand_finder() = default;

	explicit topdown_hand_finder( topdown_hand_finder::settings &s );
	void find( pacher::person &p, const cv::Mat &depth);

	settings s_;
private:
	struct contour_info{
		contour_info() = default;
		contour_info( pacher::contour&& c, double area ) : c( std::move(c) ), area(area) {};
		pacher::contour c;
		double area;
	};

	std::tuple<int, int> find_farthest( const pacher::contour &c, cv::Point from ) const;
	
};


#endif                  // include guard



