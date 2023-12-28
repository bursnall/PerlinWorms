/*
 *  CSCI 444, Advanced Computer Graphics, Spring 2021
 *
 *  Project: final project
 *  File: main.cpp
 *
 *  Description:
 *      Trying my utter and irrefutable best :)
 *      Also, worm
 *
 *  Author:
 *      Dr. Jeffrey Paone, Colorado School of Mines
 *      With assorted assistance from Treharne Bursnall, Colorado School of Mines
 *  Notes:
 *
 */

//**********************************************************************************************************************

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <cstdlib>
#include <cstdio>
#include <iostream>

#include <deque>
#include <string>
#include <set>

#include <CSCI441/OpenGLUtils.hpp>
#include <CSCI441/ShaderProgram.hpp>

//**********************************************************************************************************************
// Structure definitions

struct Vertex {
	GLfloat px, py, pz;	// point location x,y,z
	GLfloat nx, ny, nz;	// normals x,y,z
};

struct CharacterInfo {
  GLfloat advanceX; // advance.x
  GLfloat advanceY; // advance.y

  GLfloat bitmapWidth; // bitmap.width;
  GLfloat bitmapHeight; // bitmap.rows;

  GLfloat bitmapLeft; // bitmap_left;
  GLfloat bitmapTop; // bitmap_top;

  GLfloat texCoordOffsetX; // x offset of glyph in texture coordinates
} fontCharacters[128];

struct vec {
    int i;      // y
    int j;      // z
    int k;      // x
};
int getIndex( vec v ) { return (625 * v.i) + (25 * v.j) + v.k; }

class Worm {
public:
    Worm();                         // constructor, by default sets the starting position to be the cube directly facing us (24, 12, 12)
    Worm( int k, int i, int j);     // parameterized constructor
    void generate();                // function for the worm generation

    std::set<int> wormPath;         // all the nodes currently in the worm's path

    // setters and getters
    void setStart( vec start ) { this->start = start; }

private:
    std::deque<vec> heads;          // all the to-visit nodes
    int maxLength;
    int currLength = 0;
    vec start;
};

//**********************************************************************************************************************
// Global Parameters

// fix our window to a specific size
const GLint WINDOW_WIDTH = 640, WINDOW_HEIGHT = 640;

// keep track of our mouse information
GLboolean isShiftDown;                          // if the control key was pressed when the mouse was pressed
GLboolean isLeftMouseDown;                      // if the mouse left button is currently pressed
glm::vec2 mousePosition;                        // current mouse vPos

// keep track of all our camera information
struct CameraParameters {
    glm::vec3 cameraAngles;                     // cameraAngles --> x = theta, y = phi, z = radius
    glm::vec3 camDir;                           // direction to the camera
    glm::vec3 eyePoint;                         // camera vPos
    glm::vec3 lookAtPoint;                      // location of our object of interest
    glm::vec3 upVector;                         // the upVector of our camera
} arcballCam;

CSCI441::ShaderProgram *phongShaderProgram = nullptr;

// all drawing information
const GLuint NUM_VAOS = 1;
const GLuint NUM_UBOS = 3;
const struct VAOIDs {
    const GLuint CUBE = 0;
} VAOS;
struct IBOCounts {
    GLuint cube;                                // number of vertices that make up the cube
} iboCounts;
GLuint vaods[NUM_VAOS];                         // an array of our VAO descriptors
GLuint vbods[NUM_VAOS];                         // an array of our VBO descriptors
GLuint ibods[NUM_VAOS];                         // an array of our IBO descriptors
GLuint ubods[NUM_UBOS];                         // an array of our UBO descriptors
GLuint transBuffer;                             // a variable for our transform vector buffer

GLuint numInstances = 625 * 25;                 // the number of instances we are rendering
GLfloat percentWorm = 0.4f;                     // what maximum percentage we want the worm to grow to
bool justWorm = false;                          // boolean for whether to show terrain or worm
bool branchingWorm = false;                     // boolean for whether our worm is allowed to have multiple branching paths or just one
std::vector<Worm> perlinWorms;                  // our WORM(s)
std::set<int> globalWormPath;                   // in order to correctly compute numInstances, we need a set of all the worm positions
float threshold = .2f;                          // thresholds for wormies

// matrix uniforms are made in renderScene
// light uniforms except the eye pos are constant, set up those
struct LightUniforms {
    glm::vec3 lightColor = {1.0f, 1.0f, 1.0f};
    glm::vec3 lightPos = {18.0f, 12.0f, 18.0f};
} lightUniforms;

// material unforms all constant
struct MaterialUniforms {
    glm::vec3 diffColor = {0.3f, 0.4f, 0.35f};
    glm::vec3 specColor = {0.774597f, 0.774597f, 0.774597f};
    glm::vec3 ambColor = {0.15f, 0.25f, 0.15f};
    float shininess = 1.0f;
} materialUniforms;

// UBO things ! we love UBO things
GLint matrixSize, lightSize, materialSize;
GLint matrixOffsets[3], lightOffsets[3], materialOffsets[4];

// FPS information
GLdouble currentTime, lastTime;                 // used to compute elapsed time
GLuint nbFrames = 0;                            // number of frames rendered
const GLdouble FPS_SPAN = 0.33;                 // frequency to measure FPS
const GLdouble FPS_WINDOW = 5.0f;               // length of time to average FPS over
const GLuint FPS_COUNT = ceil(FPS_WINDOW / FPS_SPAN);
std::deque<GLdouble> fpsAvgs;                   // store previous FPS calculations

// Font information
GLuint fontTexture, fontVAO, fontVBO;           // stores the font texture and VAO for rendering
GLint atlasWidth, atlasHeight;                  // size of all characters in a row

CSCI441::ShaderProgram *textShaderProgram = nullptr;

struct TextShaderUniforms {
	GLint tex;                                  // Texture Map for font to apply
	GLint color;                                // Color to apply to text
} textShaderUniforms;

struct TextShaderAttributeLocations {
	GLint coord;                                // coordinate represented as (x, y, s, t)
} textShaderAttribLocs;

void wormSetup();

//**********************************************************************************************************************
// Helper Funcs

// updateCameraDirection() /////////////////////////////////////////////////////
/// \desc
/// This function updates the camera's vPos in cartesian coordinates based
///  on its vPos in spherical coordinates. Should be called every time
///  cameraAngles is updated.
///
// /////////////////////////////////////////////////////////////////////////////
void updateCameraDirection() {
    // ensure the camera does not flip upside down at either pole
    if( arcballCam.cameraAngles.y < 0 )     arcballCam.cameraAngles.y = 0.0f + 0.001f;
    if( arcballCam.cameraAngles.y >= M_PI ) arcballCam.cameraAngles.y = M_PI - 0.001f;

    // do not let our camera get too close or too far away
    if( arcballCam.cameraAngles.z <= 1.0f )  arcballCam.cameraAngles.z = 1.0f;
    if( arcballCam.cameraAngles.z >= 50.0f ) arcballCam.cameraAngles.z = 50.0f;

    // update the new direction to the camera
    arcballCam.camDir.x =  sinf( arcballCam.cameraAngles.x ) * sinf( arcballCam.cameraAngles.y );
    arcballCam.camDir.y = -cosf( arcballCam.cameraAngles.y );
    arcballCam.camDir.z = -cosf( arcballCam.cameraAngles.x ) * sinf( arcballCam.cameraAngles.y );

    // normalize this direction
    arcballCam.camDir = glm::normalize(arcballCam.camDir);
}

// calculateFPS() //////////////////////////////////////////////////////////////
/// \desc
/// This function queries the current time, increments the number of frames
///     rendered, and measures if the target time span elapsed.  If yes, then
///     calculates the Frames Per Second value and adds it to the averages
///     array.
///
// /////////////////////////////////////////////////////////////////////////////
void calculateFPS() {
    currentTime = glfwGetTime();            // query the current time
    nbFrames++;                             // add one to the number of frames rendered

    // measure if the target amount of time has elapsed
    if ( currentTime - lastTime >= FPS_SPAN ) {
        // calculate the FPS over the corresponding time span
        GLdouble currFPS = GLdouble(nbFrames)/(currentTime - lastTime);
        // add this value to the array of prior FPS values
        fpsAvgs.emplace_back( currFPS );
        // only store the last FPS_COUNT worth of values to compute average
        if(fpsAvgs.size() > FPS_COUNT) fpsAvgs.pop_front();

        // reset our FPS counters
        lastTime = currentTime;
        nbFrames = 0;
    }
}

// perlinNoise3D() //////////////////////////////////////////////////////////////
/// \desc
/// Calculates the perlin noise given 3 floats
/// Includes the gradient and fade helper functions, as well as the p table as
///     a global variable
///
// //////////////////////////////////////////////////////////////////////////////

const int p[512] = {151,160,137,91,90,15,
                    131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
                    190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
                    88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
                    77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
                    102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
                    135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
                    5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
                    223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
                    129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
                    251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
                    49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
                    138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
                    151,160,137,91,90,15,
                    131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
                    190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
                    88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
                    77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
                    102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
                    135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
                    5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
                    223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
                    129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
                    251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
                    49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
                    138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180};

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

float mix( float a, float b, float t ) {
    return a + t * (b - a);
}

float perlinNoise(float x, float y, float z) {
    int X = int(floor(x)) & 255,                    // FIND UNIT CUBE THAT
        Y = int(floor(y)) & 255,                    // CONTAINS POINT.
        Z = int(floor(z)) & 255;
    x -= floor(x);                                  // FIND RELATIVE X,Y,Z
    y -= floor(y);                                  // OF POINT IN CUBE.
    z -= floor(z);
    float u = fade(x),                              // COMPUTE FADE CURVES
          v = fade(y),                              // FOR EACH OF X,Y,Z.
          w = fade(z);
    int A = p[X  ]+Y, AA = p[A]+Z, AB = p[A+1]+Z,   // HASH COORDINATES OF
        B = p[X+1]+Y, BA = p[B]+Z, BB = p[B+1]+Z;   // THE 8 CUBE CORNERS,

    return mix(mix(mix(grad( p[AA  ], x  , y  , z   ),                          // AND ADD
                             grad( p[BA  ], x-1, y  , z   ), u ),               // BLENDED
                      mix(grad( p[AB  ], x  , y-1, z   ),                    // RESULTS
                             grad( p[BB  ], x-1, y-1, z   ), u ), v ),       // FROM  8
               mix(mix(grad( p[AA+1], x  , y  , z-1 ),               // CORNERS
                            grad( p[BA+1], x-1, y  , z-1 ), u),         // OF CUBE
                      mix(grad( p[AB+1], x  , y-1, z-1 ),
                             grad( p[BB+1], x-1, y-1, z-1 ), u ), v), w);
}

// worm() //////////////////////////////////////////////////////////////////////
/// \desc
/// Here is where I store all my wormy stuff
/// Love me some good worms
///
/// This code is HEAVILY influenced by Maxopoly's Java Simplex worm
/// https://github.com/Maxopoly/Caveworm/blob/master
///
/// A touch of clarification first, though. I designed this to work in easy
///     conjunction with the transVector generation loop. So everything in
///     the worm class is done in terms of i, j, and k in the transVector
///     loops. This just makes the conceptualizing a touch easier as well,
///     since instead of having to work in actual world coordinates, it's in
///     terms of where the particles are in relation to each other. Hence why
///     I made a little struct and comparison function - I don't want to be
///     messing those values up.
///
// /////////////////////////////////////////////////////////////////////////////
void generateWorms() {
    // important first step of CLEARING THE PREVIOUS WORM PATHS UGH
    globalWormPath.clear();

    // loops through and generates the worms
    for (auto worm = perlinWorms.begin(); worm != perlinWorms.end(); worm++ ) {
        worm->generate();
    }
}

Worm::Worm() {
    start = {12, 12, 24};
    heads.emplace_back( start );
    maxLength = percentWorm * numInstances;
}
Worm::Worm( int k, int i, int j ) {
    start = {i, j, k};
    heads.push_back( start );
    maxLength = percentWorm * numInstances;
}

float noiseInput( int pos ) {
    return (pos * 1.001f) +         // create a small float offset based on the position
           (0.5f) +                 // set ourselves in the middle of the unit square
           (pos);                   // reset ourselves back to the position
}

// alright, here's the actual real deal
void Worm::generate() {
    // in case we are doing this again after an adjustment, reset the queue and set
    heads.clear();
    wormPath.clear();

    // put our start in the queue lMAO
    heads.push_back( start );

    // go while we still have places TO go
    vec currHead;
    while( !heads.empty() ) {

        currHead = heads.front();
        if ( wormPath.count( getIndex(currHead) ) ) {
            heads.pop_front();
            continue;
        }

        // check that the worm is not colliding with other worms
        if (!branchingWorm && globalWormPath.count( getIndex( currHead ) )) { break; }

        if ( currLength >= maxLength ) { break; }

        // WOW maybe we should add the got dang point to the got dang path don't you think
        // we also need to add it to the global worm path
        wormPath.insert( getIndex(currHead) );
        globalWormPath.insert( getIndex(currHead) );

        // noise time babeyy
        // alright I goofed and they were originally all ints (?) (.)
        // TODO adjust the float values to create some normal noise values instead of whatever it is they're currently doing
//        float x = perlinNoise( noiseInput(currHead.k) * 2.401f, noiseInput(currHead.i) * 1.701f, noiseInput(currHead.j) * 1.701f );
//        float y = perlinNoise( noiseInput(currHead.k) * 1.701f, noiseInput(currHead.i) * 2.401f, noiseInput(currHead.j) * 1.701f );
//        float z = perlinNoise( noiseInput(currHead.k) * 1.701f, noiseInput(currHead.i) * 1.701f, noiseInput(currHead.j) * 2.401f );

        float x = perlinNoise( currHead.k * 2.401f, currHead.i * 1.701f, currHead.j * 1.701f );
        float y = perlinNoise( currHead.k * 1.701f, currHead.i * 2.401f, currHead.j * 1.701f );
        float z = perlinNoise( currHead.k * 1.701f, currHead.i * 1.701f, currHead.j * 2.701f );


        // test for our thresholds
        vec next = currHead;

        // x direction thresholds
        if ( x > threshold && currHead.k < 24) {
            if (branchingWorm) {
                next = {currHead.i, currHead.j, ++currHead.k};
                heads.push_back(next);
            } else { next.k++; }

        } else if ( x < -threshold && currHead.k > 0) {
            if (branchingWorm) {
                next = {currHead.i, currHead.j, --currHead.k};
                heads.push_back(next);
            } else { next.k--; }
        }
        // y direction thresholds
        if ( y > threshold && currHead.i < 24) {
            if (branchingWorm) {
                next = {++currHead.i, currHead.j, currHead.k};
                heads.push_back(next);
            } else { next.i++; }

        } else if ( y < -threshold && currHead.i > 0 ) {
            if (branchingWorm) {
                next = {--currHead.i, currHead.j, currHead.k};
                heads.push_back(next);
            } else { next.i--; }
        }
        // z direction thresholds
        if ( z > threshold && currHead.j < 24) {
            if (branchingWorm) {
                next = {currHead.i, ++currHead.j, currHead.k};
                heads.push_back(next);
            } else { next.j++; }

        } else if ( z < -threshold && currHead.j > 0 ) {
            if (branchingWorm) {
                next = {currHead.i, --currHead.j, currHead.k};
                heads.push_back(next);
            } else { next.j--; }
        }

        // if we aren't branching, need to add our updated head to the queue
        if ( !branchingWorm ) {
            heads.push_back(next);
        }

        heads.pop_front();
        currLength++;
    }
}


//**********************************************************************************************************************
// GLFW Event Callbacks

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		We will register this function as GLFW's error callback.
///	When an error within GLFW occurs, GLFW will tell us by calling
///	this function.  We can then print this info to the terminal to
///	alert the user.
///
// /////////////////////////////////////////////////////////////////////////////
static void error_callback(int error, const char* description) {
	fprintf(stderr, "[ERROR]: %s\n", description);
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		We will register this function as GLFW's keypress callback.
///	Responds to key presses and key releases
///
// /////////////////////////////////////////////////////////////////////////////
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(action == GLFW_PRESS) {
        switch(key) {
            case GLFW_KEY_ESCAPE:
            case GLFW_KEY_Q:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;

            // adjusting the thresholds
            case GLFW_KEY_MINUS: {
                // if we can go further down, do so
                if ( !(threshold - .05f < .1f) ) {
                    threshold -= .05f;
                    generateWorms();

                    wormSetup();
                }
                break;
            }
            case GLFW_KEY_EQUAL: {
                // if we can go further up, do so
                if ( !(threshold + .05f > .9f) ) {
                    threshold += .05f;
                    generateWorms();

                    wormSetup();
                }

                break;
            }

            // pressing M toggles between terrain and worm mode
            case GLFW_KEY_M:
                justWorm = !justWorm;
                wormSetup();
                break;

            // pressing L allows us to enter a new starting point
            case GLFW_KEY_L:
                fprintf( stdout, "Please enter a new worm starting point.\n");
                fflush( stdout );
                int xVal, yVal, zVal, quelWorm;
                while (true) {
                    fprintf( stdout, "Enter an x value between 0 and 24: ");
                    std::cin >> xVal;
                    fprintf( stdout, "Enter an y value between 0 and 24: ");
                    std::cin >> yVal;
                    fprintf( stdout, "Enter an z value between 0 and 24: ");
                    std::cin >> zVal;
                    fprintf( stdout, "Enter which of the worm(s) you wish to update, from 0 to %d: ", perlinWorms.size() - 1 );
                    std::cin >> quelWorm;

                    if ( !(xVal < 0 || xVal > 24 ||
                           yVal < 0 || yVal > 24 ||
                           zVal < 0 || zVal > 24) ) {
                        break;
                    }
                    if ( quelWorm >= 0 && quelWorm < perlinWorms.size() ) {
                        break;
                    }

                    fprintf( stdout, "One or more of those values were invalid, please try again.\n");
                    fflush( stdout );
                }
                perlinWorms.at(quelWorm).setStart( {xVal, yVal, zVal} );
                generateWorms();

                wormSetup();
                break;

            // pressing P switches the worm between branching off itself and not
            case GLFW_KEY_P: {
                branchingWorm = !branchingWorm;
                if (branchingWorm) {
                    perlinWorms.at(0).generate(); // only generate one branching worm, they big
                } else {
                    generateWorms();
                }

                wormSetup();
                break;
            }

            // pressing 1 and 2 makes more/less worms
            case GLFW_KEY_1:
                // if we are currently branching, don't remove worms
                // if we only have one worm, don't remove worms
                if ( branchingWorm || perlinWorms.size() == 1 ) break;

                perlinWorms.pop_back();
                generateWorms();

                wormSetup();
                break;
            case GLFW_KEY_2:
                // if we are currently branching, don't add worms
                // if we already have 6 worms, don't add any worms
                if ( branchingWorm || perlinWorms.size() == 6 ) break;

                // secondary case to set new start of the worm
                switch ( perlinWorms.size() ) {
                    case 1: // add the negative z side (12, 12, 0)
                        perlinWorms.push_back( Worm(12, 12, 0) );
                        perlinWorms.back().generate();
                        break;
                    case 2: // positive x face
                        perlinWorms.push_back( Worm(24, 12, 12) );
                        perlinWorms.back().generate();
                        break;
                    case 3: // negative x face
//                        perlinWorms.push_back( Worm(0, 12, 12) );
                        perlinWorms.push_back( Worm(12, 12, 12) );
                        perlinWorms.back().generate();
                        break;
                    case 4: // positive y face
                        perlinWorms.push_back( Worm(12, 24, 12) );
                        perlinWorms.back().generate();
                        break;
                    case 5: // negative y face
                        perlinWorms.push_back( Worm(12, 0, 12) );
                        perlinWorms.back().generate();
                        break;
                    default:
                        break;
                }

                wormSetup();
                break;

            default:
                break;
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		We will register this function as GLFW's mouse button callback.
///	Responds to mouse button presses and mouse button releases.  Keeps track if
///	the control key was pressed when a left mouse click occurs to allow
///	zooming of our arcball camera.
///
// /////////////////////////////////////////////////////////////////////////////
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if( button == GLFW_MOUSE_BUTTON_LEFT ) {
        if( action == GLFW_PRESS ) {
            isLeftMouseDown = GL_TRUE;
            isShiftDown = (mods & GLFW_MOD_SHIFT);
        } else {
            isLeftMouseDown = GL_FALSE;
            isShiftDown = GL_FALSE;
            mousePosition = glm::vec2(-9999.0f, -9999.0f);
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		We will register this function as GLFW's cursor movement callback.
///	Responds to mouse movement.  When active motion is used with the left
///	mouse button an arcball camera model is followed.
///
// /////////////////////////////////////////////////////////////////////////////
static void cursor_callback( GLFWwindow* window, double xPos, double yPos ) {
    // make sure movement is in bounds of the window
    // glfw captures mouse movement on entire screen
    if( xPos > 0 && xPos < WINDOW_WIDTH ) {
        if( yPos > 0 && yPos < WINDOW_HEIGHT ) {
            // active motion
            if( isLeftMouseDown ) {
                if( (mousePosition.x - -9999.0f) > 0.001f ) {
                    if( !isShiftDown ) {
                        // if shift is not held down, update our camera angles theta & phi
                        arcballCam.cameraAngles.x += (xPos - mousePosition.x) * 0.005f;
                        arcballCam.cameraAngles.y += (mousePosition.y - yPos) * 0.005f;
                    } else {
                        // otherwise shift was held down, update our camera radius
                        double totChgSq = (xPos - mousePosition.x) + (yPos - mousePosition.y);
                        arcballCam.cameraAngles.z += totChgSq*0.01f;
                    }
                    // recompute our camera direction
                    updateCameraDirection();
                }
                // update the last mouse vPos
                mousePosition = glm::vec2(xPos, yPos);
            }
            // passive motion
            else {

            }
            GLubyte* buffer = (GLubyte*)malloc(lightSize);
            memcpy( buffer + lightOffsets[0], &arcballCam.eyePoint[0], sizeof(glm::vec3) );
            memcpy( buffer + lightOffsets[1], &lightUniforms.lightPos[0], sizeof(glm::vec3) );
            memcpy( buffer + lightOffsets[2], &lightUniforms.lightColor[0], sizeof(glm::vec3) );

            // bind the matrix block, send to GPU
            glBindBuffer( GL_UNIFORM_BUFFER, ubods[1] );
            glBufferSubData( GL_UNIFORM_BUFFER, 0, lightSize, buffer );

            free(buffer);
            glBindBuffer( GL_UNIFORM_BUFFER, GL_NONE );
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		We will register this function as GLFW's scroll wheel callback.
///	Responds to movement of the scroll where.  Allows zooming of the arcball
///	camera.
///
// /////////////////////////////////////////////////////////////////////////////
static void scroll_callback(GLFWwindow* window, double xOffset, double yOffset ) {
    double totChgSq = yOffset;
    arcballCam.cameraAngles.z += totChgSq*0.2f;
    updateCameraDirection();
}

//**********************************************************************************************************************
// Setup Funcs

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///		Used to setup everything GLFW related.  This includes the OpenGL context
///	and our window.
///
// /////////////////////////////////////////////////////////////////////////////
GLFWwindow* setupGLFW() {
    // set what function to use when registering errors
    // this is the ONLY GLFW function that can be called BEFORE GLFW is initialized
    // all other GLFW calls must be performed after GLFW has been initialized
    glfwSetErrorCallback(error_callback);

    // initialize GLFW
    if (!glfwInit()) {
        fprintf( stderr, "[ERROR]: Could not initialize GLFW\n" );
        exit(EXIT_FAILURE);
    } else {
        fprintf( stdout, "[INFO]: GLFW initialized\n" );
    }

    glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );						// request forward compatible OpenGL context
    glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );	        // request OpenGL Core Profile context
    glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );		                // request OpenGL 4.X context
    glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 1 );		                // request OpenGL X.1 context
    glfwWindowHint( GLFW_DOUBLEBUFFER, GLFW_TRUE );                             // request double buffering
    glfwWindowHint( GLFW_RESIZABLE, GLFW_FALSE );                               // do not allow the window to be resized

    // create a window for a given size, with a given title
    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "FP: Perlin Worms", nullptr, nullptr );
    if( !window ) {						                                        // if the window could not be created, NULL is returned
        fprintf( stderr, "[ERROR]: GLFW Window could not be created\n" );
        glfwTerminate();
        exit( EXIT_FAILURE );
    } else {
        fprintf( stdout, "[INFO]: GLFW Window created\n" );
    }

    glfwMakeContextCurrent(	window );	                                        // make the created window the current window
    glfwSwapInterval( 1 );				                                // update our screen after at least 1 screen refresh

    glfwSetKeyCallback(         window, key_callback		  );            	// set our keyboard callback function
    glfwSetMouseButtonCallback( window, mouse_button_callback );	            // set our mouse button callback function
    glfwSetCursorPosCallback(	window, cursor_callback  	  );	            // set our cursor vPos callback function
    glfwSetScrollCallback(		window, scroll_callback		  );	            // set our scroll wheel callback function

    return window;										                        // return the window that was created
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Used to setup everything OpenGL related.
///
// /////////////////////////////////////////////////////////////////////////////
void setupOpenGL() {
    glEnable( GL_DEPTH_TEST );					                    // enable depth testing
    glDepthFunc( GL_LESS );							                // use less than depth test

    glFrontFace( GL_CCW );                                          // front faces are counterclockwise

    glEnable( GL_BLEND );									        // enable blending
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );	        // use one minus blending equation

    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );	// clear the frame buffer to black
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Used to initialize GLEW
///
// /////////////////////////////////////////////////////////////////////////////
void setupGLEW() {
    glewExperimental = GL_TRUE;
    GLenum glewResult = glewInit();

    // check for an error
    if( glewResult != GLEW_OK ) {
        fprintf( stderr, "[ERROR]: Error initializing GLEW\n");
        fprintf( stderr, "[ERROR]: %s\n", glewGetErrorString(glewResult) );
        exit(EXIT_FAILURE);
    } else {
        fprintf( stdout, "\n[INFO]: GLEW initialized\n" );
        fprintf( stdout, "[INFO]: Using GLEW %s\n", glewGetString(GLEW_VERSION) );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Registers our Shader Programs and query locations
///          of uniform/attribute inputs
///
// /////////////////////////////////////////////////////////////////////////////
void setupShaders() {
    // ------ Color Shader Program ------
	// load our color shader program
	phongShaderProgram        		    = new CSCI441::ShaderProgram( "shaders/phongShader.v.glsl", "shaders/phongShader.f.glsl" );

    // ------ FreeType Text Shader Program ------
    // load our text shader program
	textShaderProgram                   = new CSCI441::ShaderProgram( "shaders/freetypeColoredText.v.glsl","shaders/freetypeColoredText.f.glsl" );
    // query all of our uniform locations
	textShaderUniforms.tex              = textShaderProgram->getUniformLocation( "tex" );
    textShaderUniforms.color            = textShaderProgram->getUniformLocation( "color" );
    // query all of our attribute locations
    textShaderAttribLocs.coord          = textShaderProgram->getAttributeLocation( "coord" );
    // set static uniform values
	textShaderProgram->useProgram();
    glUniform1i( textShaderUniforms.tex, 0 );              // use Texture0

    GLfloat white[4] = {1.0f, 1.0f, 1.0f, 1.0f};            // use white text
    glUniform4fv( textShaderUniforms.color, 1, white );
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Create our VAOs & VBOs. Send vertex assets to the GPU for future rendering
///
// /////////////////////////////////////////////////////////////////////////////
void setupBuffers() {
	// generate our vertex array object descriptors
	glGenVertexArrays( NUM_VAOS, vaods );
    // generate our vertex buffer object descriptors
    glGenBuffers( NUM_VAOS, vbods );
    glGenBuffers( 1, &transBuffer );
    // generate our index buffer object descriptors
    glGenBuffers( NUM_VAOS, ibods );
    // generate our uniform buffer object descriptors
    glGenBuffers( NUM_UBOS, ubods );
    // generate our texture (framebuffer) object descriptors
//    glGenTextures( NUM_TEXS, texds );

	//------------ BEGIN cube VAO ------------

    // specify our Cube Vertex information
    const int NUM_CUBE_VERTICES = 8;
    const Vertex cubeVertices[NUM_CUBE_VERTICES] = {
            { -0.5f, -0.5f, -0.5f, -1.0f, -1.0f, -1.0f }, // 0 - bln
            { 0.5f, -0.5f, -0.5f, 1.0f, -1.0f, -1.0f },  // 1 - brn
            { 0.5f,  0.5f, -0.5f, 1.0f, 1.0f, -1.0f },  // 2 - trn
            { -0.5f, 0.5f, -0.5f, -1.0f, 1.0f, -1.0f },  // 3 - tln
            { -0.5f, -0.5f, 0.5f, -1.0f, -1.0f, 1.0f },  // 4 - blf
            { 0.5f, -0.5f, 0.5f, 1.0f, -1.0f, 1.0f },   // 5 - brf
            { 0.5f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f },    // 6 - trf
            {-0.5f, 0.5f, 0.5f, -1.0f, 1.0f, 1.0f },    // 7 - tlf
    };
    // specify our Cube Index Ordering
    const GLushort cubeIndices[] = {
            0, 2, 1,    0, 3, 2,
            1, 2, 5,    5, 2, 6,
            2, 7, 6,    3, 7, 2,
            0, 1, 4,    1, 5, 4,
            4, 5, 6,    4, 6, 7,
            0, 4, 3,    4, 7, 3,
    };
    iboCounts.cube = 36;

	// bind our Cube VAO
	glBindVertexArray( vaods[VAOS.CUBE] );

	// bind the VBO for our Cube Array Buffer
	glBindBuffer( GL_ARRAY_BUFFER, vbods[VAOS.CUBE] );
	// send the data to the GPU
	glBufferData( GL_ARRAY_BUFFER, NUM_CUBE_VERTICES * sizeof(Vertex), cubeVertices, GL_STATIC_DRAW );

    glEnableVertexAttribArray( phongShaderProgram->getAttributeLocation("vPos") );
    glEnableVertexAttribArray( phongShaderProgram->getAttributeLocation("vNorm") );
    // map the vPos attribute to data within our buffer
    glVertexAttribPointer( phongShaderProgram->getAttributeLocation("vPos"), 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) 0 );
    glVertexAttribPointer( phongShaderProgram->getAttributeLocation("vNorm"), 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) (3 * sizeof(GLfloat)));

    // bind the VBO for our Cube Element Array Buffer
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, ibods[VAOS.CUBE] );
	// send the data to the GPU
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, iboCounts.cube * sizeof(GLushort), cubeIndices, GL_STATIC_DRAW );

	//------------  END  cube VAO ------------

    //------------ START   UBOs    ------------
    GLubyte* buffer = nullptr; // create a buffer to hold all our buffers

    // matrix ubo
    // get all our indices for the UBOs
    GLint matrixBlockIndex = glGetUniformBlockIndex(phongShaderProgram->getShaderProgramHandle(), "MatrixBlock" );
    // query the sizes of the blocks
    glGetActiveUniformBlockiv(phongShaderProgram->getShaderProgramHandle(), matrixBlockIndex,
                              GL_UNIFORM_BLOCK_DATA_SIZE, &matrixSize );
    // create our buffers
    // get our offsets in each block
    GLchar *matrixNames[3] = {"mvpMtx", "modelMtx", "normalMtx"};
    GLuint matrixIndices[3];
    glGetUniformIndices(phongShaderProgram->getShaderProgramHandle(), 3, matrixNames, matrixIndices );
    glGetActiveUniformsiv(phongShaderProgram->getShaderProgramHandle(), 3,
                          matrixIndices, GL_UNIFORM_OFFSET, matrixOffsets );

    // setup fake data initially, to be changed in renderScene()
    glm::mat4 mvpMtx{1.0f}, modelMtx{1.0f};
    glm::mat4 normMtx{1.0f};

    // set buffer, send fake data
    buffer = (GLubyte*)malloc(matrixSize);
    memcpy( buffer + matrixOffsets[0], &mvpMtx[0][0], sizeof(glm::mat4) );
    memcpy( buffer + matrixOffsets[1], &modelMtx[0][0], sizeof(glm::mat4) );
    memcpy( buffer + matrixOffsets[2], &normMtx[0][0], sizeof(glm::mat4) );

    // bind the matrix block, send to GPU
    glBindBuffer( GL_UNIFORM_BUFFER, ubods[0] );
    glBufferData( GL_UNIFORM_BUFFER, matrixSize, buffer, GL_DYNAMIC_DRAW );
    // bind to binding qualifier 0
    glBindBufferBase( GL_UNIFORM_BUFFER, 0, ubods[0] );
    phongShaderProgram->setUniformBlockBinding( "MatrixBlock", 0);

    // free buffer for next uniform block
    free(buffer); buffer = nullptr;

    // light ubo
    // now for the light block
    GLint lightBlockIndex = glGetUniformBlockIndex(phongShaderProgram->getShaderProgramHandle(), "LightBlock" );
    glGetActiveUniformBlockiv(phongShaderProgram->getShaderProgramHandle(), lightBlockIndex,
                              GL_UNIFORM_BLOCK_DATA_SIZE, &lightSize );
    GLchar *lightNames[3] = {"eyePos", "lightPos", "lightColor"};
    GLuint lightIndices[3];
    glGetUniformIndices(phongShaderProgram->getShaderProgramHandle(), 3, lightNames, lightIndices );
    glGetActiveUniformsiv(phongShaderProgram->getShaderProgramHandle(), 3,
                          lightIndices, GL_UNIFORM_OFFSET, lightOffsets );

    // setup our (real!1!!) data, set buffer, send data
    glm::vec3 eyePos{arcballCam.eyePoint}, lightPos{lightUniforms.lightPos}, lightColor{lightUniforms.lightColor};
    buffer = (GLubyte*)malloc(lightSize);
    memcpy( buffer + lightOffsets[0], &eyePos[0], sizeof(glm::vec3) );
    memcpy( buffer + lightOffsets[1], &lightPos[0], sizeof(glm::vec3) );
    memcpy( buffer + lightOffsets[2], &lightColor[0], sizeof(glm::vec3) );

    // bind the light block
    glBindBuffer( GL_UNIFORM_BUFFER, ubods[1] );
    glBufferData( GL_UNIFORM_BUFFER, lightSize, buffer, GL_DYNAMIC_DRAW );
    // bind to binding spot 1
    glBindBufferBase( GL_UNIFORM_BUFFER, 1, ubods[1] );
    phongShaderProgram->setUniformBlockBinding( "LightBlock", 1);

    // free the buffer for the materials
    free(buffer); buffer = nullptr;

    // material ubo
    // and now the material block
    GLint materialBlockIndex = glGetUniformBlockIndex(phongShaderProgram->getShaderProgramHandle(), "MaterialBlock" );
    glGetActiveUniformBlockiv(phongShaderProgram->getShaderProgramHandle(), materialBlockIndex,
                              GL_UNIFORM_BLOCK_DATA_SIZE, &materialSize );
    GLchar *materialNames[4] = {"materialDiffColor", "materialSpecColor",
                                "materialShininess", "materialAmbColor"};
    GLuint materialIndices[4];
    glGetUniformIndices(phongShaderProgram->getShaderProgramHandle(), 4, materialNames, materialIndices );
    glGetActiveUniformsiv(phongShaderProgram->getShaderProgramHandle(), 4,
                          materialIndices, GL_UNIFORM_OFFSET, materialOffsets );

    // set some more REAL (!!!1!1) data, set buffer, send across the pond
    glm::vec3 diff{materialUniforms.diffColor}, spec{materialUniforms.specColor}, amb{materialUniforms.ambColor};
    float shine = materialUniforms.shininess;
    buffer = (GLubyte*)malloc(materialSize);
    memcpy( buffer + materialOffsets[0], &diff[0], sizeof(glm::vec3) );
    memcpy( buffer + materialOffsets[1], &spec[0], sizeof(glm::vec3) );
    memcpy( buffer + materialOffsets[2], &shine, sizeof(float) );
    memcpy( buffer + materialOffsets[3], &amb[0], sizeof(glm::vec3) );

    // bind the material block
    glBindBuffer( GL_UNIFORM_BUFFER, ubods[2] );
    glBufferData( GL_UNIFORM_BUFFER, materialSize, buffer, GL_DYNAMIC_DRAW );
    // bind to binding spot 2
    glBindBufferBase( GL_UNIFORM_BUFFER, 2, ubods[2] );
    phongShaderProgram->setUniformBlockBinding( "MaterialBlock", 2);

    // unbind our buffer in preparation
    glBindBuffer( GL_UNIFORM_BUFFER, 0 );

    //------------  END    UBOs    ------------
}

void wormSetup() {
    //------------ START trans buf ------------
    numInstances = 625 * 25;
    glm::vec4 transVecs[numInstances];
    int count = 0; // since we are no longer

    // nested for loops (i PROMISE it run fast) (it doesn't) to calculate the trans vecs for everything
    for ( int i = 0; i < numInstances / 625; i++ ) { // y
        for ( int j = 0; j < 25; j++ ) { // z
            for ( int k = 0; k < 25; k++ ) { // x

                // if this i, j, k vector is in the worm path, don't render it
                vec currParticle = {i, j, k};

                // reminder of cases for whether we are branching or not
                if ( branchingWorm ) {
                    // checking for the branching paths
                    if ((!justWorm && perlinWorms.front().wormPath.count(getIndex(currParticle))) ||
                        (justWorm && !perlinWorms.front().wormPath.count(getIndex(currParticle)))) {
                        continue;
                    }
                } else {
                    // checking for the non-branching paths
                    if ((!justWorm && globalWormPath.count(getIndex(currParticle))) ||
                        (justWorm && !globalWormPath.count(getIndex(currParticle)))) {
                        continue;
                    }
                }

                transVecs[count] = glm::vec4( -12.f + k,
                                              -12.f + i,
                                              -12.f + j,
                                              0.f);
                count++;
            }
        }
    }

    // for correct rendering, set numInstances correctly
    if (branchingWorm) {
        if ( justWorm ) numInstances = perlinWorms.front().wormPath.size();
        else            numInstances -= perlinWorms.front().wormPath.size();
    } else {
        if ( justWorm ) numInstances = globalWormPath.size();
        else            numInstances -= globalWormPath.size();
    }

    glBindVertexArray( vaods[VAOS.CUBE] );

    // buffer for the instanced translation variable
    GLint transVecLoc = phongShaderProgram->getAttributeLocation("transVec");

    glBindBuffer( GL_ARRAY_BUFFER, transBuffer );
    glBufferData( GL_ARRAY_BUFFER, numInstances * sizeof(glm::vec4), &transVecs[0], GL_STATIC_DRAW );

    glEnableVertexAttribArray( transVecLoc );
    glVertexAttribPointer( transVecLoc, 4, GL_FLOAT, GL_FALSE, 0, NULL );
    glVertexAttribDivisor( transVecLoc, 1 );

    //------------  END trans buf  ------------
}


// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Load in a font face from a ttf file, store the glyphs in a VAO+VBO w/ texture
///
// /////////////////////////////////////////////////////////////////////////////
void setupFonts() {
	FT_Library ftLibrary;
    FT_Face fontFace;

	if(FT_Init_FreeType(&ftLibrary)) {
        fprintf(stderr, "[ERROR]: Could not init FreeType library\n");
        exit(EXIT_FAILURE);
	} else {
	    fprintf(stdout, "[INFO]: FreeType library initialized\n");
	}

    const std::string FONT_FILE_NAME = "assets/fonts/DroidSansMono.ttf";
	if(FT_New_Face( ftLibrary, FONT_FILE_NAME.c_str(), 0, &fontFace)) {
        fprintf(stderr, "[ERROR]: Could not open font %s\n", FONT_FILE_NAME.c_str());
        exit(EXIT_FAILURE);
	} else {
	    fprintf(stdout, "[INFO]; Successfully loaded font face \"%s\"\n", FONT_FILE_NAME.c_str());
	}

	FT_Set_Pixel_Sizes( fontFace, 0, 20);

	FT_GlyphSlot g = fontFace->glyph;
	GLuint w = 0;
	GLuint h = 0;

	for(int i = 32; i < 128; i++) {
        if(FT_Load_Char(fontFace, i, FT_LOAD_RENDER)) {
            fprintf(stderr, "[ERROR]: Loading character %c failed!\n", i);
            continue;
        }

        w += g->bitmap.width;
        h = (h > g->bitmap.rows ? h : g->bitmap.rows);
	}

	// you might as well save this value as it is needed later on
	atlasWidth = w;
    atlasHeight = h;

    // create texture memory to store the glyphs for rendering
	glEnable( GL_TEXTURE_2D );
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &fontTexture);
	glBindTexture( GL_TEXTURE_2D, fontTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	GLint x = 0;

	for(int i = 32; i < 128; i++) {
        if(FT_Load_Char( fontFace, i, FT_LOAD_RENDER))
            continue;

        fontCharacters[i].advanceX = g->advance.x >> 6;
        fontCharacters[i].advanceY = g->advance.y >> 6;

        fontCharacters[i].bitmapWidth = g->bitmap.width;
        fontCharacters[i].bitmapHeight = g->bitmap.rows;

        fontCharacters[i].bitmapLeft = g->bitmap_left;
        fontCharacters[i].bitmapTop = g->bitmap_top;

        fontCharacters[i].texCoordOffsetX = (float)x / w;

        glTexSubImage2D(GL_TEXTURE_2D, 0, x, 0, g->bitmap.width, g->bitmap.rows, GL_RED, GL_UNSIGNED_BYTE, g->bitmap.buffer);

        x += g->bitmap.width;
	}

    fprintf(stdout, "[INFO]: Font face texture atlas setup\n");

	// create the VAO + VBO to ultimately store the text to be displayed
	glGenVertexArrays( 1, &fontVAO );
	glBindVertexArray( fontVAO );
	glGenBuffers( 1, &fontVBO );
	glBindBuffer( GL_ARRAY_BUFFER, fontVBO);
	glEnableVertexAttribArray( textShaderAttribLocs.coord );
	glVertexAttribPointer( textShaderAttribLocs.coord, 4, GL_FLOAT, GL_FALSE, 0, (void*)0 );

	fprintf(stdout, "[INFO]: Font face buffer setup\n");

	FT_Done_Face(fontFace);
	FT_Done_FreeType( ftLibrary);
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Initialize all of our scene information here
///
// /////////////////////////////////////////////////////////////////////////////
void setupScene() {
    // set up mouse info
    isLeftMouseDown = GL_FALSE;
    isShiftDown = GL_FALSE;
    mousePosition = glm::vec2( -9999.0f, -9999.0f );

    // set up camera info
    arcballCam.cameraAngles   = glm::vec3( 1.82f, 2.01f, 32.0f );
    arcballCam.camDir         = glm::vec3( -1.0f, -1.0f, -1.0f );
    arcballCam.lookAtPoint    = glm::vec3( 0.0f, 0.0f, 0.0f) ;
    arcballCam.upVector       = glm::vec3( 0.0f,  1.0f,  0.0f );
    updateCameraDirection();

    // initialize FPS timers to be non-zero
    currentTime = lastTime = glfwGetTime();
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Create our OpenGL context,
///          load all information to the GPU,
///          initialize scene information
///
// /////////////////////////////////////////////////////////////////////////////
GLFWwindow* initialize() {
    // GLFW sets up our OpenGL context so must be done first
    GLFWwindow* window = setupGLFW();	                // initialize all the GLFW specific information related to OpenGL and our window
    setupGLEW();										// initialize all the GLEW specific information
    setupOpenGL();										// initialize all the OpenGL specific information

    CSCI441::OpenGLUtils::printOpenGLInfo();            // print our OpenGL information

    setupShaders();                                     // load all of our shader programs onto the GPU and get shader input locations

    perlinWorms.push_back( Worm() );                    // construct and generate our Perlin Worm(s)
    generateWorms();

    setupBuffers();										// load all our VAOs and VBOs onto the GPU
    wormSetup();                                        // load the worm VAO specifically
    setupFonts();                                       // load all our fonts as a VAO to the GPU
    setupScene();                                       // initialize all of our scene information

    fprintf( stdout, "\n[INFO]: Setup complete\n" );

    return window;
}

//**********************************************************************************************************************
// Cleanup Functions

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Delete shaders off of the GPU
///
// /////////////////////////////////////////////////////////////////////////////
void cleanupShaders() {
    fprintf( stdout, "[INFO]: ...deleting shaders.\n" );

    delete phongShaderProgram;
    delete textShaderProgram;
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Delete VAOs and VBOs off of the GPU
///
// /////////////////////////////////////////////////////////////////////////////
void cleanupBuffers() {
    fprintf( stdout, "[INFO]: ...deleting IBOs....\n" );
    glDeleteBuffers( NUM_VAOS, ibods );

    fprintf( stdout, "[INFO]: ...deleting VBOs....\n" );
    glDeleteBuffers( NUM_VAOS, vbods );
    glDeleteBuffers( 1, &transBuffer );

    fprintf( stdout, "[INFO]: ...deleting VAOs....\n" );
    glDeleteVertexArrays( NUM_VAOS, vaods );

    fprintf( stdout, "[INFO]: ...deleting UBOs....\n" );
    glDeleteBuffers( NUM_UBOS, ubods );
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Delete Fonts off of the GPU
///
// /////////////////////////////////////////////////////////////////////////////
void cleanupFonts() {
    fprintf( stdout, "[INFO]: ...deleting fonts...\n" );

    glDeleteBuffers( 1, &fontVBO );
    glDeleteVertexArrays( 1, &fontVAO );
    glDeleteTextures( 1, &fontTexture );
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Free all memory on the CPU/GPU and close our OpenGL context
///
// /////////////////////////////////////////////////////////////////////////////
void shutdown(GLFWwindow* window) {
    fprintf( stdout, "\n[INFO]: Shutting down.......\n" );
    fprintf( stdout, "[INFO]: ...closing window...\n" );
    glfwDestroyWindow( window );                        // close our window
    cleanupShaders();                                   // delete shaders from GPU
    cleanupBuffers();                                   // delete VAOs/VBOs from GPU
    cleanupFonts();                                     // delete VAOs/VBOs/textures from GPU
    fprintf( stdout, "[INFO]: ...closing GLFW.....\n" );
    glfwTerminate();						            // shut down GLFW to clean up our context
    fprintf( stdout, "[INFO]: ..shut down complete!\n" );
}

//**********************************************************************************************************************
// Rendering / Drawing Functions - this is where the magic happens!

// /////////////////////////////////////////////////////////////////////////////
/// \desc
/// Displays a given text string, using the corresponding character map, starting
///     at a given (x, y) coordinate.  Each character is scaled by
///     (scaleX, scaleY).
///
// /////////////////////////////////////////////////////////////////////////////
void renderText( const char *text, CharacterInfo characters[], float x, float y, float scaleX, float scaleY ) {
	const GLuint TEXT_LENGTH = strlen(text);    // the number of characters in the text string
	const GLuint NUM_POINTS = 6 * TEXT_LENGTH;  // each character is drawn as a quad of two triangles
    glm::vec4 coords[NUM_POINTS];               // values correspond to vertex attributes x, y, s, t

	GLint n = 0;

	for(const char *p = text; *p; p++) {
		int characterIndex = (int)*p;

		CharacterInfo character = characters[characterIndex];

		GLfloat characterXPos = x + character.bitmapLeft * scaleX;
		GLfloat characterYPos = -y - character.bitmapTop * scaleY;
		GLfloat scaledCharacterWidth = character.bitmapWidth * scaleX;
		GLfloat scaledCharacterHeight = character.bitmapHeight * scaleY;
		GLfloat glyphWidth = character.bitmapWidth / atlasWidth;
		GLfloat glyphHeight = character.bitmapHeight / atlasHeight;

		// Advance the cursor to the start of the next character
		x += character.advanceX * scaleX;
		y += character.advanceY * scaleY;

		// Skip glyphs that have no pixels
		if( !scaledCharacterWidth || !scaledCharacterHeight)
			continue;

		coords[n++] = glm::vec4( characterXPos, -characterYPos, character.texCoordOffsetX, 0);
		coords[n++] = glm::vec4( characterXPos + scaledCharacterWidth, -characterYPos    , character.texCoordOffsetX + glyphWidth, 0);
		coords[n++] = glm::vec4( characterXPos, -characterYPos - scaledCharacterHeight, character.texCoordOffsetX, glyphHeight); //remember: each glyph occupies a different amount of vertical space

		coords[n++] = glm::vec4( characterXPos + scaledCharacterWidth, -characterYPos    , character.texCoordOffsetX + glyphWidth, 0);
		coords[n++] = glm::vec4( characterXPos, -characterYPos - scaledCharacterHeight, character.texCoordOffsetX, glyphHeight);
		coords[n++] = glm::vec4( characterXPos + scaledCharacterWidth, -characterYPos - scaledCharacterHeight, character.texCoordOffsetX + glyphWidth, glyphHeight);
	}

	glBindVertexArray(fontVAO);
	glBindBuffer(GL_ARRAY_BUFFER, fontVBO);
	glBufferData(GL_ARRAY_BUFFER, NUM_POINTS * sizeof( glm::vec4 ), coords, GL_DYNAMIC_DRAW);
	glDrawArrays(GL_TRIANGLES, 0, n);
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
/// Handles the drawing of everything to our buffer.  Needs the view and
///     projection matrices to apply as part of the ModelViewProjection matrix.
///
// /////////////////////////////////////////////////////////////////////////////
void renderScene( const glm::mat4 VIEW_MTX, const glm::mat4 PROJ_MTX ) {
	if (!phongShaderProgram) return;

    // use our Color Shading Program
	phongShaderProgram->useProgram();

    // create our Model, View, Projection matrices
    glm::mat4 modelMtx = glm::mat4( 1.0f);

    // precompute the modelViewProjection matrix
    glm::mat4 mvpMtx = PROJ_MTX * VIEW_MTX * modelMtx;

    // precompute the normal matrix
//    glm::mat4 normalMatrix = glm::transpose(glm::inverse(modelMtx));
    glm::mat4 normalMatrix = glm::mat4( 1.0f);

	// update our modelViewProjection matrix
	mvpMtx = PROJ_MTX * VIEW_MTX * modelMtx;

    GLubyte* buffer = (GLubyte*)malloc(matrixSize);

    // update our dynamic matrix values for the cube
    memcpy( buffer + matrixOffsets[0], &mvpMtx[0][0], sizeof(glm::mat4) );
    memcpy( buffer + matrixOffsets[1], &modelMtx[0][0], sizeof(glm::mat4) );
    memcpy( buffer + matrixOffsets[2], &normalMatrix[0][0], sizeof(glm::mat4) );
    // bind the matrix block, send to GPU
    glBindBuffer( GL_UNIFORM_BUFFER, ubods[0] );
    glBufferSubData( GL_UNIFORM_BUFFER, 0, matrixSize, buffer );
    // free buffer, unbind ubo
    free(buffer); buffer = nullptr;
    glBindBuffer( GL_UNIFORM_BUFFER, 0 );
	
	// bind our Cube VAO
	glBindVertexArray( vaods[VAOS.CUBE] );
	// draw our cube!
//	glDrawElements( GL_TRIANGLES, iboCounts.cube, GL_UNSIGNED_SHORT, (void*)0 );
    glDrawElementsInstanced( GL_TRIANGLES, iboCounts.cube, GL_UNSIGNED_SHORT, (void*) 0, numInstances);
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Update all of our scene objects - perform animation here
///
// /////////////////////////////////////////////////////////////////////////////
void updateScene() {

}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Print the formatted FPS information to the screen
///
// /////////////////////////////////////////////////////////////////////////////
void printFPS( GLint windowWidth, GLint windowHeight ) {
    GLdouble currFPS = 0, fpsAvg = 0, totalFPS = 0.0;

    // calculate the average FPS
    for( const auto& fps : fpsAvgs ) {
        totalFPS += fps;
    }
    if( !fpsAvgs.empty() ) {
        currFPS = fpsAvgs.back();
        fpsAvg = totalFPS / fpsAvgs.size();
    }

    char fpsStr[80];
    sprintf( fpsStr, "%.3f frames/sec (Avg: %.3f)", currFPS, fpsAvg);

    textShaderProgram->useProgram();
    glBindTexture(GL_TEXTURE_2D, fontTexture);
    glBindVertexArray(fontVAO);

    GLfloat sx = 2.0 / (GLfloat)windowWidth;
    GLfloat sy = 2.0 / (GLfloat)windowHeight;

    renderText( fpsStr, fontCharacters, -1 + 8 * sx, 1 - 30 * sy, sx, sy );
}

// /////////////////////////////////////////////////////////////////////////////
/// \desc
///      Runs our draw loop and renders/updates our scene
/// \param window - window to render the scene to
// /////////////////////////////////////////////////////////////////////////////
void run(GLFWwindow* window) {
    // NOTE: we are doing the Viewport and Projection calculations prior to the loop since we are not
    // allowing our window to be resized so these values will never change throughout the life of our
    // program.  If we allowed the window to be resized, then we would need to create a window resize
    // callback and update these values only on change.

    // Get the size of our framebuffer.  Ideally this should be the same dimensions as our window, but
    // when using a Retina display the actual window can be larger than the requested window.  Therefore
    // query what the actual size of the window we are rendering to is.
    GLint windowWidth, windowHeight;
    glfwGetFramebufferSize( window, &windowWidth, &windowHeight );

    // update the viewport - tell OpenGL we want to render to the whole window
    glViewport( 0, 0, windowWidth, windowHeight );

    // compute our projection matrix
    const glm::mat4 PROJ_MTX = glm::perspective( 45.0f, windowWidth / (GLfloat) windowHeight, 0.001f, 100.0f );

    //  This is our draw loop - all rendering is done here.  We use a loop to keep the window open
    //	until the user decides to close the window and quit the program.  Without a loop, the
    //	window will display once and then the program exits.
    while( !glfwWindowShouldClose(window) ) {	        // check if the window was instructed to be closed
        // clear the prior contents of our buffer
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // compute our view matrix based on our current camera setup
        glm::mat4 vMtx = glm::lookAt( arcballCam.lookAtPoint + arcballCam.camDir * arcballCam.cameraAngles.z,
                                      arcballCam.lookAtPoint,
                                      arcballCam.upVector );

        renderScene( vMtx, PROJ_MTX );                  // render our scene

        calculateFPS();                                 // compute current Frames Per Second
        printFPS( windowWidth, windowHeight );          // display FPS information on screen

        glfwSwapBuffers(window);                        // flush the OpenGL commands and make sure they get rendered!
        glfwPollEvents();				                // check for any events and signal to redraw screen

        updateScene();                                  // update the objects in our scene
    }
}

//**********************************************************************************************************************

// program entry point
int main() {
    GLFWwindow *window = initialize();                  // create OpenGL context and setup EVERYTHING for our program
    run(window);                                        // enter our draw loop and run our program
    shutdown(window);                                   // free up all the memory used and close OpenGL context
    return EXIT_SUCCESS;				                // exit our program successfully!
}
