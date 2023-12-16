#version 410 core

// ***** VERTEX SHADER UNIFORMS *****
uniform Transform {
    mat4 mvpMtx;
    float time;
};

// ***** VERTEX SHADER INPUT *****
in vec3 vPos;     // vertex position

// ***** VERTEX SHADER OUTPUT *****
out vec2 tex;

// ***** VERTEX SHADER SUBROUTINES *****


// ***** VERTEX SHADER HELPER FUNCTIONS *****


// ***** VERTEX SHADER MAIN FUNCTION *****
void main() {
    // transform our vertex into clip space
    gl_Position = mvpMtx * vec4( vPos, 1);
    tex = vec2(cos(time) + vPos.x, sin(time) + vPos.z);
}
