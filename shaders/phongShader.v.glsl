#version 410 core

// uniform inputs
uniform MatrixBlock {
    mat4 mvpMtx;// the precomputed Model-View-Projection Matrix
    mat4 modelMtx;// just the model matrix
    mat4 normalMtx;// normal matrix
};

uniform LightBlock {
    vec3 eyePos;                    // eye position in world space
    vec3 lightPos;                  // light position in world space
    vec3 lightColor;                // light color
};

// varying inputs
layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vNorm;
layout(location = 2) in vec4 transVec;

// outputs
out vec3 position;
out vec3 normal;
out vec3 lightVector;
out vec3 viewVector;

void main() {
    normal = normalize( mat3(normalMtx) * normalize(vNorm) );
    position = ( modelMtx * ( vec4(vPos, 1.0f) + transVec )).xyz;

    lightVector = normalize(lightPos - position);
    viewVector = normalize(eyePos - position);

    gl_Position = mvpMtx * ( vec4(vPos, 1.0f) + transVec );
}