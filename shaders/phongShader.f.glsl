#version 410 core

// uniform inputs
uniform LightBlock {
    vec3 eyePos;                    // eye position in world space
    vec3 lightPos;                  // light position in world space
    vec3 lightColor;                // light color
};

uniform MaterialBlock {
    vec3 materialDiffColor;         // the material diffuse color
    vec3 materialSpecColor;         // the material specular color
    float materialShininess;        // the material shininess value
    vec3 materialAmbColor;          // the material ambient color
};

// attribute inputs
in vec3 position;      // the position of this specific fragment in object space
in vec3 normal;   // the normal of this specific fragment in object space
in vec3 lightVector;
in vec3 viewVector;

// varying outputs
layout(location = 0) out vec4 fragColorOut;    // color to apply to this vertex


// compute the diffuse color using lambertian diffuse reflectance
vec3 diffuseColor(vec3 fragPosition, vec3 fragNormal) {
//    vec3 lightVector = normalize(lightBlock.lightPos - fragPosition);
    vec3 diffColor = lightColor * materialDiffColor * max( dot(fragNormal, normalize(lightVector)), 0.0 );

    return diffColor;
}

// compute the specular color using Blinn-Phong specular reflectance
vec3 halfwaySpec(vec3 fragPosition, vec3 fragNormal) {
    // calculate our light vector
//    vec3 lightVector = normalize(lightBlock.lightPos - fragPosition); // s

    // get the halfway vector
//    vec3 viewVector = normalize(lightBlock.eyePos - fragPosition); // v
    vec3 halfwayVector = normalize(normalize(viewVector) + normalize(lightVector)); // h

    // if not in reflection area, no spec component dot(s, n) != 0
    vec3 specColor = vec3(0.0f, 0.0f, 0.0f);
    if ( max( dot(normalize(lightVector), fragNormal), 0.0f) > 0.0f) {
        // compute the specular component (L * M * pow(max(dot(h, n), 0), alpha)
        specColor = lightColor * materialSpecColor * pow(max( dot(fragNormal, halfwayVector), 0.0f ), 4.0f * materialShininess);
    }

    return specColor;
}

void main() {
    // compute each component of the Phong Illumination Model
    if ( gl_FrontFacing ) {
        vec3 diffColor = diffuseColor(position, normalize(normal));
        vec3 specColor = halfwaySpec(position, normalize(normal));
        vec3 ambColor = lightColor * materialAmbColor;

        // assign the final color for this vertex
        fragColorOut = vec4(diffColor + specColor + ambColor, 1.0f);
    } else {
        fragColorOut = vec4(lightColor * materialAmbColor, 1.0f);
    }
}