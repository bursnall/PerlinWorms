/*
 *  CSCI 444, Advanced Computer Graphics, Spring 2021
 *
 *  File: freetypeColoredText.v.glsl
 *
 *  Description:
 *      Shader to display text to the screen
 *
 *  Author:
 *      Dr. Jeffrey Paone, Colorado School of Mines
 *
 *  Notes:
 *
 */

// we are using OpenGL 4.1 Core profile
#version 410 core

// ***** UNIFORMS *****


// ***** ATTRIBUTES *****
in vec4 coord;

// ***** VERTEX SHADER OUTPUT *****
out vec2 vTexCoord;

void main() {
    gl_Position = vec4( coord.xy, 0.0f, 1.0f ); // first two components correspond to 2D vertex position
    vTexCoord = coord.zw;                       // next two components correspond to texture coordinate
}
