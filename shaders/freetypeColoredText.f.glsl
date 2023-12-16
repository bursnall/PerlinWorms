/*
 *  CSCI 444, Advanced Computer Graphics, Spring 2021
 *
 *  File: freetypeColoredText.f.glsl
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
uniform sampler2D tex;
uniform vec4 color;

// ***** FRAGMENT SHADER INPUT *****
in vec2 vTexCoord;

// ***** FRAGMENT SHADER OUTPUT *****
out vec4 fragColorOut;

void main() {
    vec4 texel = texture(tex, vTexCoord);   // texture is B/W for off/on of each pixel to make the character
    
    fragColorOut = vec4(1.0f, 1.0f, 1.0f, texel.r) * color;
}
