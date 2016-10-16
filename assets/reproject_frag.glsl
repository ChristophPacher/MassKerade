#version 420

uniform sampler2D rgbKinect;

// Input attributes
in VertexData {
    highp vec4 position;
	highp vec3 normal;
	highp vec2 texCoords0;
	highp float depth;
} inD;

out vec4 color;

// Kernel
void main( void )
{
	color.a = inD.depth;
	color.rgb = texture2D( rgbKinect, inD.texCoords0.st ).rgb;
}
