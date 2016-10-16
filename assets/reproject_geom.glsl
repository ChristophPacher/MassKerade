#version 420
precision highp float;

uniform highp mat4 ciModelViewProjection;

uniform highp vec3 mesh_threshold;

layout(triangles) in;
in VertexData {
    highp vec4 position;
	highp vec3 normal;
	highp vec2 texCoords0;
	highp float depth;
} inD[3];

// Outputs
layout (triangle_strip, max_vertices=3) out;
out VertexData {
    highp vec4 position;
	highp vec3 normal;
	highp vec2 texCoords0;
	highp float depth;
} outD;

void main()
{
	int len = gl_in.length();	
	for(int i = 0; i < len; ++i)
	{	
		// all points with no depth are dropped 
		if (inD[i].position.z != 0 ){
			int tail = i;
			int head = i+1;
			if (head >= len) head = 0;
			highp vec4 edge = inD[head].position - inD[tail].position;
			
			// all points that form a edge, that is too long are dropped
			if ( edge.x * edge.x < mesh_threshold.x &&
				 edge.y * edge.y < mesh_threshold.y &&
				 edge.z * edge.z < mesh_threshold.z ) 
			{ 
				outD.position = inD[i].position;
				outD.normal = inD[i].normal;
				outD.texCoords0 = inD[i].texCoords0;
				outD.depth = inD[i].depth;
				
				outD.position = ciModelViewProjection * outD.position;
				gl_Position = outD.position;

				// done with the vertex
				EmitVertex();
			}
		} 
	}
	// the triangle is finished. if not 3 vertices were emitted the triangle is dropped
	EndPrimitive();

}
