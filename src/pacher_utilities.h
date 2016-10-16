/*!
 * @file	pacher_utilities.h
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

#ifndef _H_PACHER_UTILITIES
#define _H_PACHER_UTILITIES


#include <vector>

#include <string>



namespace myUtilities {
	struct TransformFactors{
		float mul, add;
	};
	/*!
	 * @brief		Calculates Transform from one range to another range
	 * 
	 * 
	 * 
	 * @param: 		float fromMin
	 * @param: 		float fromMax
	 * @param: 		float toMin
	 * @param: 		float toMax
	 * @return:   	std::vector<float> [0] == conversion multiplier [1] == conversion offset
	 */
	TransformFactors calcDataRangeTransformFactors(float fromMin, float fromMax, float toMin, float toMax);


	void printGLErrors( std::string location );

	inline float fast_length( float x, float y ){
		return x * x + y * y;
	};
}

#endif