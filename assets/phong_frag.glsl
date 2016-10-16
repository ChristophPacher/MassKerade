 #version 150 
  
 uniform vec3 	eye_pos_w;
 uniform vec4 	light_pos_w;
  
 in vec4 	pos_w;
 in vec3 	normal_w;
  
 out vec4 oColor; 
  
 void main(void) { 
 
	vec3 n = normalize(normal_w);
		
	vec3 l = normalize(vec3( light_pos_w.xyz - pos_w.xyz * light_pos_w.w )); 
	vec3 v = eye_pos_w - pos_w.xyz;
	vec3 v_norm = normalize( v );
	vec3 r = reflect( -l, n );	
	
	float NdotL = max( dot(n, l), 0.0 );
	
	vec3 color = vec3(0, 0, 0);
	
	if (NdotL > 0 ) {
		color +=   vec3( 0.5, 0.5, 0.5 ) * NdotL + vec3( 1, 1, 1 ) * pow( max( dot(v_norm, r), 0.0 ), 80);
	}
	
	
	oColor = vec4(color, 1); 
 } 