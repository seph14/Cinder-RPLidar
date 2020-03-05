#version 330 core

uniform mat4 ciModelViewProjection;
in vec4		 ciPosition;
in int		 vInstanceIdx;

uniform float 		  uSize;
uniform float 		  uScale;
uniform samplerBuffer uBuffer;

void main( void ){
	vec2 pos 	= uScale * texelFetch(uBuffer, vInstanceIdx).xy;
	gl_Position	= ciModelViewProjection * vec4(uSize * ciPosition.xy + pos, 0.0, 1.0);
}
