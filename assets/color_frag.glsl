 #version 420 
  
 in vec3 color;
  
 out vec4 oColor; 
  
 void main(void) { 
	oColor = vec4(color, 1); 
 } 