 #version 420 
  
 uniform mat4	ciModelViewProjection; 
 uniform mat4 	ciModelMatrix;
 uniform mat3 	ciModelMatrixInverseTranspose; 
 
 uniform vec3 	colorA;
 uniform vec3	colorB;
 uniform vec3 	eye_pos_w;
 uniform vec4 	light_pos_w;
 uniform vec4 	light_color;
 uniform vec3	ambient_color;
 
 uniform float 	num_verts;
 
 in vec4		ciPosition; 
 in vec3		ciNormal; 
 
 
 out vec3 	color;
  
 void main(void) { 
	vec4 pos_w = ciModelMatrix * ciPosition;
	vec3 n = normalize( ciModelMatrixInverseTranspose * ciNormal );
	
	vec3 l = normalize(vec3( light_pos_w.xyz - pos_w.xyz * light_pos_w.w )); 
	vec3 v = eye_pos_w - pos_w.xyz;
	vec3 v_norm = normalize( v );
	vec3 r = reflect( -l, n );
	
	float NdotL = max( dot(n, l), 0.0 );
	
	color = ambient_color;
	
	if (NdotL > 0 ) {
		float dist = length(v);
		//float light_r = 1.0;
		//float cutoff = 0.005;
		//float att = 1.0 / pow( dist / light_r + 1, 2); 
		//att = max((att - cutoff) / (1 - cutoff), 0);
		float att = 1;
		color +=  att * ( mix( colorA, colorB, gl_VertexID / num_verts)  * NdotL + light_color.rgb * pow( max( dot(v_norm, r), 0.0 ), light_color.a));
	}

 	gl_Position = ciModelViewProjection * ciPosition; 
 } 