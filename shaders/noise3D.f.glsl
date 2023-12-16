#version 410 core

// ***** FRAGMENT SHADER UNIFORMS *****
uniform PermutationTable {
    int p[512];
};
uniform Transform {
    mat4 mvpMtx;
    float time;
};

// ***** FRAGMENT SHADER INPUT *****
in vec2 tex;

// ***** FRAGMENT SHADER OUTPUT *****
out vec4 fragColorOut;

// ***** FRAGMENT SHADER SUBROUTINES *****


// ***** FRAGMENT SHADER HELPER FUNCTIONS *****
float fade(float time) {
    return (6 * pow(time, 5)) - (15 * pow(time, 4)) + (10 * pow(time, 3));
}

float grad(int hash, float x, float y, float z) {
    switch(hash & 0xF)
    {
        case 0x0: return  x + y;
        case 0x1: return -x + y;
        case 0x2: return  x - y;
        case 0x3: return -x - y;
        case 0x4: return  x + z;
        case 0x5: return -x + z;
        case 0x6: return  x - z;
        case 0x7: return -x - z;
        case 0x8: return  y + z;
        case 0x9: return -y + z;
        case 0xA: return  y - z;
        case 0xB: return -y - z;
        case 0xC: return  y + x;
        case 0xD: return -y + z;
        case 0xE: return  y - x;
        case 0xF: return -y - z;
        default: return 0; // never happens
    }
}

float noise(float x, float y) {
    int X = int(floor(x)) & 255,                    // FIND UNIT CUBE THAT
    Y = int(floor(y)) & 255,                        // CONTAINS POINT.
    Z = int(floor(z)) & 255;
    x -= floor(x);                                  // FIND RELATIVE X,Y,Z
    y -= floor(y);                                  // OF POINT IN CUBE.
    z -= floor(z);
    double u = fade(x),                             // COMPUTE FADE CURVES
    v = fade(y),                                    // FOR EACH OF X,Y,Z.
    w = fade(z);
    int A = p[X  ]+Y, AA = p[A]+Z, AB = p[A+1]+Z,   // HASH COORDINATES OF
    B = p[X+1]+Y, BA = p[B]+Z, BB = p[B+1]+Z;       // THE 8 CUBE CORNERS,

    return mix( mix( mix( grad( p[AA  ], x  , y  , z   ),             // AND ADD
                          grad( p[BA  ], x-1, y  , z   ), u ),        // BLENDED
                     mix( grad( p[AB  ], x  , y-1, z   ),             // RESULTS
                          grad( p[BB  ], x-1, y-1, z   ), u ), v ),   // FROM  8
                mix( mix( grad( p[AA+1], x  , y  , z-1 ),             // CORNERS
                          grad( p[BA+1], x-1, y  , z-1 ), u),         // OF CUBE
                     mix( grad( p[AB+1], x  , y-1, z-1 ),
                          grad( p[BB+1], x-1, y-1, z-1 ), u ), v), w);
}

// ***** FRAGMENT SHADER MAIN FUNCTION *****
void main() {
    // generate our noise
    float n = noise(tex.s + cos(time + tex.t), tex.t + sin(time + tex.s));

    vec3 sky = vec3( mix(0.3f, 1.0f, n),
    mix(0.3f, 1.0f, n),
    mix(0.9f, 1.0f, n) );

    // set the output color to the interpolated color
    fragColorOut = vec4(sky, 1.0f);

    // if viewing the backside of the fragment,
    // reverse the colors as a visual cue
    if( !gl_FrontFacing ) {
        fragColorOut.rgb = fragColorOut.bgr;
    }
}
