 #version 150
  
 uniform mat4	ciModelViewProjection; 
 uniform mat4 	ciModelMatrix;
 uniform mat3 	ciModelMatrixInverseTranspose; 
 

 in vec4		ciPosition; 
 in vec3		ciNormal; 
 
 
 out vec4 	pos_w;
 out vec3 	normal_w;
  
 void main(void) { 
	pos_w = ciModelMatrix * ciPosition;
	normal_w = ciModelMatrixInverseTranspose * ciNormal;
 	gl_Position = ciModelViewProjection * ciPosition; 
 } 