/*!
 * @file	pacher_utilities.cpp
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

#include "pacher_utilities.h"
#include "math.h"


#include <cinder/gl/gl.h>
#include <gl/glu.h>
#include "cinder/app/AppBasic.h"

using namespace ci;
using namespace ci::app;
using namespace std;

myUtilities::TransformFactors myUtilities::calcDataRangeTransformFactors( float fromMin, float fromMax, float toMin, float toMax )
{
	myUtilities::TransformFactors returnVal;
	returnVal.mul = abs(toMax - toMin)/abs(fromMax - fromMin);
	returnVal.add = toMin - fromMin * returnVal.mul;
	return returnVal;
}

void myUtilities::printGLErrors( std::string location )
{
	GLenum errCode;
	const GLubyte *errString;
	int num = 1;
	while ( ( errCode = glGetError() ) != GL_NO_ERROR ) {
		errString = gluErrorString( errCode );
		console() << location << " OpenGL Error Code " << num << ": " << errCode << " " << errString << endl;
		++num;
	}
}

