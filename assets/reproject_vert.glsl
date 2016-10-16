#version 420
precision highp float;

uniform highp mat3 ciNormalMatrix;

uniform highp vec4 xMul_yMul_zMul_zAdd;

in highp vec4 ciPosition; 
in highp vec3 ciNormal; 
in highp vec2 ciTexCoord0; 

uniform usampler2D depthKinect;

// Outs
out VertexData {
    highp vec4 position;
	highp vec3 normal;
	highp vec2 texCoords0;
	highp float depth;
} outD;

// Kernel
void main( void )
{
	outD.texCoords0 = ciTexCoord0;

	outD.normal = ciNormalMatrix * ciNormal;
	
	outD.depth = float(texelFetch( depthKinect, ivec2( ciPosition.xy ), 0 ).r);
	
	// from mm to m
	highp float depth_meter = 0.001 *  outD.depth;
	// transform 16 bit range to min max 8bit range and invert values so close values to the
	// camera are now big and far away is small
	outD.depth = 1 - (outD.depth * xMul_yMul_zMul_zAdd.z + xMul_yMul_zMul_zAdd.w)/255;
	
	ivec2 d_size = textureSize(depthKinect, 0);
	
	highp vec4 p = ciPosition;
	//shader assumes all points are in a plane defined by the points (0/0/0) and (w/h/0)
	p.x = (( p.x / ( d_size.x - 1 ) ) - 0.5f ) * depth_meter * xMul_yMul_zMul_zAdd.x;
	p.y = (( p.y / ( d_size.y - 1 ) )- 0.5f ) * depth_meter * xMul_yMul_zMul_zAdd.y;
	p.z = -depth_meter;
	outD.position = p;
	gl_Position = p;
}
