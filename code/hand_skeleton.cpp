// hand teliko 

// Include C++ headers
#include <iostream>
#include <string>
#include <map>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Shader loading utilities and other
#include <common/shader.h>
#include <common/util.h>
#include <common/camera.h>
#include <common/model.h>
#include <common/skeleton.h>

using namespace std;
using namespace glm;

// Function prototypes
void initialize();
void createContext();
void mainLoop();
void free();
struct Light; struct Material;
void uploadMaterial(const Material& mtl);
void uploadLight(const Light& light);
map<int, mat4> calculateModelPoseFromCoordinates(map<int, float> q);
vector<mat4> calculateSkinningTransformations(map<int, float> q);
vector<float> calculateSkinningIndices();

#define W_WIDTH 1024
#define W_HEIGHT 768
#define TITLE "Lab 06"
#define pi 3.1415926

// global variables
GLFWwindow* window;
Camera* camera;
GLuint shaderProgram;
GLuint projectionMatrixLocation, viewMatrixLocation, modelMatrixLocation;
// light properties
GLuint LaLocation, LdLocation, LsLocation, lightPositionLocation, lightPowerLocation;
// material properties
GLuint KdLocation, KsLocation, KaLocation, NsLocation;

GLuint surfaceVAO, surfaceVerticesVBO, surfacesBoneIndecesVBO, maleBoneIndicesVBO;
Drawable* segment, * skeletonSkin, * sk;
GLuint useSkinningLocation, boneTransformationsLocation;
Skeleton* skeleton;

struct Light {
    glm::vec4 La;
    glm::vec4 Ld;
    glm::vec4 Ls;
    glm::vec3 lightPosition_worldspace;
    float power;
};

struct Material {
    glm::vec4 Ka;
    glm::vec4 Kd;
    glm::vec4 Ks;
    float Ns;
};

const Material boneMaterial{
    vec4{ 0.1, 0.1, 0.1, 1 },
    vec4{ 1.0, 1.0, 1.0, 1 },
    vec4{ 0.3, 0.3, 0.3, 1 },
    0.1f
};

Light light{
    vec4{ 1, 1, 1, 1 },
    vec4{ 1, 1, 1, 1 },
    vec4{ 1, 1, 1, 1 },
    vec3{ 0, 4, 4 },
    20.0f
};

// Coordinate names for mnemonic indexing
enum CoordinateName {
    H11_X = 0, H12_X, H21_X, H22_X, H23_X, H31_X, H32_X, H33_X, H41_X, H42_X, H43_X, H51_X, H52_X, H53_X, DOFS
};
// Joint names for mnemonic indexing
enum JointName {
    H11 = 0, H12, H21, H22, H23, H31, H32, H33, H41, H42, H43, H51, H52, H53, JOINTS
};

// default pose used for binding the skeleton and the mesh
static const map<int, float> bindingPose = {
    {CoordinateName::H11_X,  0.0f},
    {CoordinateName::H12_X, 0.0f},
    {CoordinateName::H21_X, 0.0f},
    {CoordinateName::H22_X,  0.0f},
    {CoordinateName::H23_X,  0.0f},
    {CoordinateName::H31_X,  0.0f},
    {CoordinateName::H32_X, 3.0f},
    {CoordinateName::H33_X, 3.0f},
    {CoordinateName::H41_X, -5.0f},
    {CoordinateName::H42_X, 5.0f},
    {CoordinateName::H43_X, 0.0f},
    {CoordinateName::H51_X, 0.0f},
    {CoordinateName::H52_X, -15.0f},
    {CoordinateName::H53_X, -15.0f}
};

void uploadMaterial(const Material& mtl) {
    glUniform4f(KaLocation, mtl.Ka.r, mtl.Ka.g, mtl.Ka.b, mtl.Ka.a);
    glUniform4f(KdLocation, mtl.Kd.r, mtl.Kd.g, mtl.Kd.b, mtl.Kd.a);
    glUniform4f(KsLocation, mtl.Ks.r, mtl.Ks.g, mtl.Ks.b, mtl.Ks.a);
    glUniform1f(NsLocation, mtl.Ns);
}

void uploadLight(const Light& light) {
    glUniform4f(LaLocation, light.La.r, light.La.g, light.La.b, light.La.a);
    glUniform4f(LdLocation, light.Ld.r, light.Ld.g, light.Ld.b, light.Ld.a);
    glUniform4f(LsLocation, light.Ls.r, light.Ls.g, light.Ls.b, light.Ls.a);
    glUniform3f(lightPositionLocation, light.lightPosition_worldspace.x,
        light.lightPosition_worldspace.y, light.lightPosition_worldspace.z);
    glUniform1f(lightPowerLocation, light.power);
}

map<int, mat4> calculateModelPoseFromCoordinates(map<int, float> q) {
    map<int, mat4> jointLocalTransformations;

    // h11
    mat4 h11RotX = rotate(mat4(), radians(q[CoordinateName::H11_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H11] = h11RotX;

    // h12
    mat4 h12RotX = rotate(mat4(), radians(q[CoordinateName::H12_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H12] = h12RotX;

    // h21
    mat4 h21RotX = rotate(mat4(), radians(q[CoordinateName::H21_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H21] = h21RotX;

    // h22
    mat4 h22RotX = rotate(mat4(), radians(q[CoordinateName::H22_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H22] = h22RotX;

    // h23
    mat4 h23RotX = rotate(mat4(), radians(q[CoordinateName::H23_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H23] = h23RotX;

    // h31
    mat4 h31RotX = rotate(mat4(), radians(q[CoordinateName::H31_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H31] = h31RotX;

    // h32
    mat4 h32RotX = rotate(mat4(), radians(q[CoordinateName::H32_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H32] = h32RotX;

    // h33
    mat4 h33RotX = rotate(mat4(), radians(q[CoordinateName::H33_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H33] = h33RotX;

    // h41
    mat4 h41RotX = rotate(mat4(), radians(q[CoordinateName::H41_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H41] = h41RotX;

    // h42
    mat4 h42RotX = rotate(mat4(), radians(q[CoordinateName::H42_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H42] = h42RotX;

    // h43
    mat4 h43RotX = rotate(mat4(), radians(q[CoordinateName::H43_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H43] = h43RotX;

    // h51
    mat4 h51RotX = rotate(mat4(), radians(q[CoordinateName::H51_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H51] = h51RotX;

    // h52
    mat4 h52RotX = rotate(mat4(), radians(q[CoordinateName::H52_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H52] = h52RotX;

    // h53
    mat4 h53RotX = rotate(mat4(), radians(q[CoordinateName::H53_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H53] = h53RotX;



    return jointLocalTransformations;
}

vector<mat4> calculateSkinningTransformations(map<int, float> q) {
    auto jointLocalTransformationsBinding = calculateModelPoseFromCoordinates(bindingPose);
    skeleton->setPose(jointLocalTransformationsBinding);
    auto bindingWorldTransformations = skeleton->getJointWorldTransformations();

    auto jointLocalTransformationsCurrent = calculateModelPoseFromCoordinates(q);
    skeleton->setPose(jointLocalTransformationsCurrent);
    auto currentWorldTransformations = skeleton->getJointWorldTransformations();

    vector<mat4> skinningTransformations(JointName::JOINTS);
    for (auto joint : bindingWorldTransformations) {
        mat4 BInvWorld = glm::inverse(joint.second);
        mat4 JWorld = currentWorldTransformations[joint.first];
        skinningTransformations[joint.first] = JWorld * BInvWorld;
    }
    return skinningTransformations;
}

vector<float> calculateSkinningIndices() {
    // Task 4.3: assign a body index for each vertex in the model (skin) based
    // on its proximity to a body part (e.g. tight)
    vector<float> indices;
    for (auto v : skeletonSkin->indexedVertices) {
        if (v.x < -0.13 && v.y < -0.23) {
            indices.push_back(JointName::H12);
        }
        else if (v.x < -0.13) {
            indices.push_back(JointName::H11);
        }
        else if (v.x > -0.13 && v.y < -0.385) {
            indices.push_back(JointName::H23);
        }
        else if (v.x > -0.13 && v.y < -0.345) {
            indices.push_back(JointName::H22);
        }
        else if (v.x > -0.13 && v.y < -0.24) {
            indices.push_back(JointName::H21);
        }
        else if (v.x > -0.06 && v.y < -0.395) {
            indices.push_back(JointName::H33);
        }
        else if (v.x > -0.06 && v.y < -0.35) {
            indices.push_back(JointName::H32);
        }
        else if (v.x > -0.06 && v.y < -0.24) {
            indices.push_back(JointName::H31);
        }
        else if (v.x > 0.013 && v.y < -0.38) {
            indices.push_back(JointName::H43);
        }
        else if (v.x > 0.013 && v.y < -0.34) {
            indices.push_back(JointName::H42);
        }
        else if (v.x > 0.013 && v.y < -0.24) {
            indices.push_back(JointName::H41);
        }
        else if (v.x > 0.07 && v.y < -0.33) {
            indices.push_back(JointName::H53);
        }
        else if (v.x > 0.07 && v.y < -0.3) {
            indices.push_back(JointName::H52);
        }
        else if (v.x > 0.07 && v.y < -0.24) {
            indices.push_back(JointName::H51);
        }

    }
    return indices;
}

void createContext() {
    // shader
    shaderProgram = loadShaders(
        "StandardShading.vertexshader",
        "StandardShading.fragmentshader");

    // get pointers to uniforms
    modelMatrixLocation = glGetUniformLocation(shaderProgram, "M");
    viewMatrixLocation = glGetUniformLocation(shaderProgram, "V");
    projectionMatrixLocation = glGetUniformLocation(shaderProgram, "P");
    KaLocation = glGetUniformLocation(shaderProgram, "mtl.Ka");
    KdLocation = glGetUniformLocation(shaderProgram, "mtl.Kd");
    KsLocation = glGetUniformLocation(shaderProgram, "mtl.Ks");
    NsLocation = glGetUniformLocation(shaderProgram, "mtl.Ns");
    LaLocation = glGetUniformLocation(shaderProgram, "light.La");
    LdLocation = glGetUniformLocation(shaderProgram, "light.Ld");
    LsLocation = glGetUniformLocation(shaderProgram, "light.Ls");
    lightPositionLocation = glGetUniformLocation(shaderProgram, "light.lightPosition_worldspace");
    lightPowerLocation = glGetUniformLocation(shaderProgram, "light.power");
    useSkinningLocation = glGetUniformLocation(shaderProgram, "useSkinning");
    boneTransformationsLocation = glGetUniformLocation(shaderProgram, "boneTransformations");

    float xx = -0.3f;

    vector<vec3> segmentVertices = {
        vec3(-0.4, xx, 0.0f),
        vec3(0.2, xx, 0.0f)
    };

    segment = new Drawable(segmentVertices);



    // Task 3.1a: define the relations between the bodies and the joints
    // A skeleton is a collection of joints and bodies. Each body is independent
    // of each other (conceptually). Furthermore, each body can  have many
    // drawables (geometries) attached. The joints are related to each other
    // and form a parent child relations. A joint is attached on a body.
    skeleton = new Skeleton(modelMatrixLocation, viewMatrixLocation, projectionMatrixLocation);

    // h11
    Joint* h11Joint = new Joint(); // creates a joint
    h11Joint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::H11] = h11Joint; // adds the joint in the skeleton's dictionary
    // h12
    Joint* h12Joint = new Joint();
    h12Joint->parent = h11Joint;
    skeleton->joints[JointName::H12] = h12Joint;
    // h21
    Joint* h21Joint = new Joint(); // creates a joint
    h21Joint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::H21] = h21Joint; // adds the joint in the skeleton's dictionary
    // h22
    Joint* h22Joint = new Joint();
    h22Joint->parent = h21Joint;
    skeleton->joints[JointName::H22] = h22Joint;
    // h23
    Joint* h23Joint = new Joint();
    h23Joint->parent = h22Joint;
    skeleton->joints[JointName::H23] = h23Joint;
    // h31
    Joint* h31Joint = new Joint(); // creates a joint
    h31Joint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::H31] = h31Joint; // adds the joint in the skeleton's dictionary
    // h32
    Joint* h32Joint = new Joint();
    h32Joint->parent = h31Joint;
    skeleton->joints[JointName::H32] = h32Joint;
    // h33
    Joint* h33Joint = new Joint();
    h33Joint->parent = h32Joint;
    skeleton->joints[JointName::H33] = h33Joint;
    // h41
    Joint* h41Joint = new Joint(); // creates a joint
    h41Joint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::H41] = h41Joint; // adds the joint in the skeleton's dictionary
    // h42
    Joint* h42Joint = new Joint();
    h42Joint->parent = h41Joint;
    skeleton->joints[JointName::H42] = h42Joint;
    // h43
    Joint* h43Joint = new Joint();
    h43Joint->parent = h42Joint;
    skeleton->joints[JointName::H43] = h43Joint;
    // h51
    Joint* h51Joint = new Joint(); // creates a joint
    h51Joint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::H51] = h51Joint; // adds the joint in the skeleton's dictionary
    // h52
    Joint* h52Joint = new Joint();
    h52Joint->parent = h51Joint;
    skeleton->joints[JointName::H52] = h52Joint;
    // h53
    Joint* h53Joint = new Joint();
    h53Joint->parent = h52Joint;
    skeleton->joints[JointName::H53] = h53Joint;

    // skin
    skeletonSkin = new Drawable("models/h1.obj");
    //sk = new Drawable("models/h1.obj");
    auto maleBoneIndices = calculateSkinningIndices();
    glGenBuffers(1, &maleBoneIndicesVBO);
    glBindBuffer(GL_ARRAY_BUFFER, maleBoneIndicesVBO);
    glBufferData(GL_ARRAY_BUFFER, maleBoneIndices.size() * sizeof(float),
        &maleBoneIndices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(3);
}

void free() {
    delete segment;
    // the skeleton owns the bodies and joints so memory is freed when skeleton
    // is deleted
    delete skeleton;
    delete skeletonSkin;
    //delete sk;

    glDeleteBuffers(1, &surfaceVAO);
    glDeleteVertexArrays(1, &surfaceVerticesVBO);
    glDeleteVertexArrays(1, &surfacesBoneIndecesVBO);

    glDeleteVertexArrays(1, &maleBoneIndicesVBO);

    glDeleteProgram(shaderProgram);
    glfwTerminate();
}

void mainLoop() {
    camera->position = vec3(0, -0.3, 1);
    do {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        // camera
        camera->update();
        mat4 projectionMatrix = camera->projectionMatrix;
        mat4 viewMatrix = camera->viewMatrix;

        // light
        uploadLight(light);

        glUniform1i(useSkinningLocation, 0);
        glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
        glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);





        // Task 3.2: assign values to the generalized coordinates and correct
        // the transformations in calculateModelPoseFromCoordinates()
        // Homework 2: add 3 rotational DoFs for the pelvis and the necessary
        // DoFs for left leg.
        // Task 3.3: make the skeleton walk (approximately)
        // Homework 3: model Michael Jackson's Moonwalk .
        //*/
        float time = glfwGetTime();
        int w = 6.25;

        map<int, float> q;
        q[CoordinateName::H11_X] = 0;
        q[CoordinateName::H12_X] = 0;
        q[CoordinateName::H21_X] = 0;
        q[CoordinateName::H22_X] = 0;
        q[CoordinateName::H23_X] = 0;
        q[CoordinateName::H31_X] = 0;
        q[CoordinateName::H32_X] = 0;
        q[CoordinateName::H33_X] = 0;
        q[CoordinateName::H41_X] = 0;
        q[CoordinateName::H42_X] = 0;
        q[CoordinateName::H43_X] = 0;
        q[CoordinateName::H51_X] = 0;
        q[CoordinateName::H52_X] = 0;
        q[CoordinateName::H53_X] = 0;


        auto jointLocalTransformations = calculateModelPoseFromCoordinates(q);
        skeleton->setPose(jointLocalTransformations);

        glUniform1i(useSkinningLocation, 0);
        uploadMaterial(boneMaterial);
        skeleton->draw(viewMatrix, projectionMatrix);
        //*/

        /*/--
        sk->bind();
        mat4 maleModelMatrix2 = mat4(1);
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &maleModelMatrix2[0][0]);
        glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
        glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        sk->draw();
        //*/
        // Task 4.1: draw the skin using wireframe mode
        //*/
        skeletonSkin->bind();

        mat4 maleModelMatrix = glm::translate(mat4(), vec3(0.0f, 0.0f, 0.0f));;
        //mat4 maleModelMatrix = glm::rotate(mat4(), -3.14f / 2.0f, vec3(0.0f, 1.0f, 0.0f));
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &maleModelMatrix[0][0]);
        glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
        glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);

        // Task 4.2: calculate the bone transformations
        auto T = calculateSkinningTransformations(q);
        glUniformMatrix4fv(boneTransformationsLocation, T.size(),
            GL_FALSE, &T[0][0][0]);

        glUniform1i(useSkinningLocation, 1);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        skeletonSkin->draw();

        //----------------------------------------------------------------------------------------
        // first segment
        segment->bind();
        mat4 bone1 = mat4(1);
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &bone1[0][0]);
        // draw segment
        segment->draw(GL_LINES);
        segment->draw(GL_POINTS);
        //----------------------------------------------------------------------------------------------

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        //*/

        glfwSwapBuffers(window);
        glfwPollEvents();
    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
        glfwWindowShouldClose(window) == 0);
}

void initialize() {
    // Initialize GLFW
    if (!glfwInit()) {
        throw runtime_error("Failed to initialize GLFW\n");
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(W_WIDTH, W_HEIGHT, TITLE, NULL, NULL);
    if (window == NULL) {
        glfwTerminate();
        throw runtime_error(string(string("Failed to open GLFW window.") +
            " If you have an Intel GPU, they are not 3.3 compatible." +
            "Try the 2.1 version.\n"));
    }
    glfwMakeContextCurrent(window);

    // Start GLEW extension handler
    glewExperimental = GL_TRUE;

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        throw runtime_error("Failed to initialize GLEW\n");
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Hide the mouse and enable unlimited movement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, W_WIDTH / 2, W_HEIGHT / 2);

    // Gray background color
    glClearColor(0.5f, 0.5f, 0.5f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    // glEnable(GL_CULL_FACE);

    // enable point size when drawing points
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Log
    logGLParameters();

    // Create camera
    camera = new Camera(window);
}

int main(void) {
    try {
        initialize();
        createContext();
        mainLoop();
        free();
    }
    catch (exception& ex) {
        cout << ex.what() << endl;
        getchar();
        free();
        return -1;
    }

    return 0;
}