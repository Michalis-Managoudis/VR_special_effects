// telik;o gia neo modelo

// Include C++ headers
#include <iostream>
#include <windows.h>
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
    B0_T_Z = 0, B0_R_Y, B1_T_Z, B1_R_Y, B1_R_X, F1R_R_X, F1L_R_X, F2R_R_X, F2L_R_X, F3R_R_X, F3L_R_X, H1R_R_Y, H1L_R_Y, H1R_R_Z, H1L_R_Z, H2R_R_Y, H2L_R_Y, DOFS
};
// Joint names for mnemonic indexing
enum JointName {
    B0 = 0, B1, F1R, F1L, F2R, F2L, F3R, F3L, H1R, H1L, H2R, H2L, JOINTS
};

// default pose used for binding the skeleton and the mesh
static const map<int, float> bindingPose = {
    {CoordinateName::B0_T_Z, 0.0f},
    {CoordinateName::B0_R_Y, 0.0f},
    {CoordinateName::B1_T_Z, 0.0f},
    {CoordinateName::B1_R_Y, 0.0f},
    {CoordinateName::B1_R_X, 0.0f},
    {CoordinateName::F1R_R_X, 0.0f},
    {CoordinateName::F1L_R_X, 0.0f},
    {CoordinateName::F2R_R_X, 0.0f},
    {CoordinateName::F2L_R_X, 0.0f},
    {CoordinateName::F3R_R_X, 0.0f},
    {CoordinateName::F3L_R_X, 0.0f},
    {CoordinateName::H1R_R_Y, 0.0f},
    {CoordinateName::H1L_R_Y, 0.0f},
    {CoordinateName::H1R_R_Z, 0.0f},
    {CoordinateName::H1L_R_Z, 0.0f},
    {CoordinateName::H2R_R_Y, 0.0f},
    {CoordinateName::H2L_R_Y, 0.0f}
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

    // base / B0 joint
    mat4 baseTra = translate(mat4(), vec3(0.0f, 0.0f, q[CoordinateName::B0_T_Z]));
    mat4 baseRotY = rotate(mat4(), radians(q[CoordinateName::B0_R_Y]), vec3(0, 1, 0));
    jointLocalTransformations[JointName::B0] = baseTra * baseRotY;

    // chest / B1 joint
    mat4 chestTra = translate(mat4(), vec3(0.0f, 0.0f, q[CoordinateName::B1_T_Z]));
    mat4 chestRotX = rotate(mat4(), radians(q[CoordinateName::B1_R_X]), vec3(1, 0, 0));
    mat4 chestRotY = rotate(mat4(), radians(q[CoordinateName::B1_R_Y]), vec3(0, 1, 0));
    jointLocalTransformations[JointName::B1] = chestTra * chestRotX * chestRotY;

    // right foot / F1R joint
    mat4 f1rRotX = rotate(mat4(), radians(q[CoordinateName::F1R_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F1R] = f1rRotX;

    // left foot / F1L joint
    mat4 f1lRotX = rotate(mat4(), radians(q[CoordinateName::F1L_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F1L] = f1lRotX;

    // right foot / F2R joint
    mat4 f2rRotX = rotate(mat4(), radians(q[CoordinateName::F2R_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F2R] = f2rRotX;

    // left foot / F2L joint
    mat4 f2lRotX = rotate(mat4(), radians(q[CoordinateName::F2L_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F2L] = f2lRotX;

    // right foot / F3R joint
    mat4 f3rRotX = rotate(mat4(), radians(q[CoordinateName::F3R_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F3R] = f3rRotX;

    // left foot / F3L joint
    mat4 f3lRotX = rotate(mat4(), radians(q[CoordinateName::F3L_R_X]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::F3L] = f3lRotX;

    // right hand / H1R joint H1R_R_Z
    mat4 h1rRotZ = rotate(mat4(), radians(q[CoordinateName::H1R_R_Z]), vec3(1, 0, 0));
    mat4 h1rRotY = rotate(mat4(), radians(q[CoordinateName::H1R_R_Y]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H1R] = h1rRotY * h1rRotZ;

    // left hand / H1L joint
    mat4 h1lRotZ = rotate(mat4(), radians(q[CoordinateName::H1L_R_Z]), vec3(1, 0, 0));
    mat4 h1lRotY = rotate(mat4(), radians(q[CoordinateName::H1L_R_Y]), vec3(1, 0, 0));
    jointLocalTransformations[JointName::H1L] = h1lRotY * h1lRotZ;

    // right hand / H2R joint
    mat4 h2rRotY = rotate(mat4(), radians(q[CoordinateName::H2R_R_Y]), vec3(0, 1, 0));
    jointLocalTransformations[JointName::H2R] = h2rRotY;

    // left hand / H2L joint
    mat4 h2lRotY = rotate(mat4(), radians(q[CoordinateName::H2L_R_Y]), vec3(0, 1, 0));
    jointLocalTransformations[JointName::H2L] = h2lRotY;

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
    // assign a body index for each vertex in the model (skin) based
    // on its proximity to a body part (e.g. tight)
    vector<float> indices;
    for (auto v : skeletonSkin->indexedVertices)
    {
        if (v.y <= 0.27 && v.x < 0.0) {
            indices.push_back(JointName::F3R);
        }
        else if (v.y <= 0.27 && v.x > 0.0) {
            indices.push_back(JointName::F3L);
        }
        else if (v.y <= 1.8 && v.x < 0.0) {
            indices.push_back(JointName::F2R);
        }
        else if (v.y <= 1.8 && v.x > 0.0) {
            indices.push_back(JointName::F2L);
        }
        else if (v.y <= 3.1 && v.x < 0.0) {
            indices.push_back(JointName::F1R);
        }
        else if (v.y <= 3.1 && v.x > 0.0) {
            indices.push_back(JointName::F1L);
        }
        else if (v.x <= -1.42) {
            indices.push_back(JointName::H2R);
        }
        else if (v.x >= 1.42) {
            indices.push_back(JointName::H2L);
        }
        else if (v.x <= -0.6) {
            indices.push_back(JointName::H1R);
        }
        else if (v.x >= 0.6) {
            indices.push_back(JointName::H1L);
        }
        else if (v.y >= 3.5) {
            indices.push_back(JointName::B1);
        }
        else {
            indices.push_back(JointName::B0);
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

    float xx = -2.22f;

    vector<vec3> segmentVertices = {
        vec3(xx, 4, 0.0f),
        vec3(xx, 5.5, 0.0f)
    };

    segment = new Drawable(segmentVertices);

    // Task 3.1a: define the relations between the bodies and the joints
    // A skeleton is a collection of joints and bodies. Each body is independent
    // of each other (conceptually). Furthermore, each body can  have many
    // drawables (geometries) attached. The joints are related to each other
    // and form a parent child relations. A joint is attached on a body.
    skeleton = new Skeleton(modelMatrixLocation, viewMatrixLocation, projectionMatrixLocation);

    // base B0
    Joint* baseJoint = new Joint(); // creates a joint
    baseJoint->parent = NULL; // assigns the parent joint (NULL -> no parent)
    skeleton->joints[JointName::B0] = baseJoint; // adds the joint in the skeleton's dictionary

    // chest B1
    Joint* chestJoint = new Joint();
    chestJoint->parent = baseJoint;
    skeleton->joints[JointName::B1] = chestJoint;

    // right foot up F1R
    Joint* f1rJoint = new Joint();
    f1rJoint->parent = baseJoint;
    skeleton->joints[JointName::F1R] = f1rJoint;

    // left foot up F1L
    Joint* f1lJoint = new Joint();
    f1lJoint->parent = baseJoint;
    skeleton->joints[JointName::F1L] = f1lJoint;

    // right foot middle F2R
    Joint* f2rJoint = new Joint();
    f2rJoint->parent = f1rJoint;
    skeleton->joints[JointName::F2R] = f2rJoint;

    // left foot middle F2L
    Joint* f2lJoint = new Joint();
    f2lJoint->parent = f1lJoint;
    skeleton->joints[JointName::F2L] = f2lJoint;

    // right foot down F3R
    Joint* f3rJoint = new Joint();
    f3rJoint->parent = f2rJoint;
    skeleton->joints[JointName::F3R] = f3rJoint;

    // left foot down F3L
    Joint* f3lJoint = new Joint();
    f3lJoint->parent = f2lJoint;
    skeleton->joints[JointName::F3L] = f3lJoint;

    // right hand up H1R
    Joint* h1rJoint = new Joint();
    h1rJoint->parent = chestJoint;
    skeleton->joints[JointName::H1R] = h1rJoint;

    // left hand up H1L
    Joint* h1lJoint = new Joint();
    h1lJoint->parent = chestJoint;
    skeleton->joints[JointName::H1L] = h1lJoint;

    // right hand down H2R
    Joint* h2rJoint = new Joint();
    h2rJoint->parent = h1rJoint;
    skeleton->joints[JointName::H2R] = h2rJoint;

    // left hand down H2L
    Joint* h2lJoint = new Joint();
    h2lJoint->parent = h1lJoint;
    skeleton->joints[JointName::H2L] = h2lJoint;

    // skin
    skeletonSkin = new Drawable("models/human.obj");
    sk = new Drawable("models/human.obj");
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
    delete sk;
    glDeleteBuffers(1, &surfaceVAO);
    glDeleteVertexArrays(1, &surfaceVerticesVBO);
    glDeleteVertexArrays(1, &surfacesBoneIndecesVBO);

    glDeleteVertexArrays(1, &maleBoneIndicesVBO);

    glDeleteProgram(shaderProgram);
    glfwTerminate();
}

void mainLoop() {
    camera->position = vec3(0, 3, 7);
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
        q[CoordinateName::B0_T_Z] = 0;
        q[CoordinateName::B0_R_Y] = 0;
        q[CoordinateName::B1_T_Z] = sin(time);
        q[CoordinateName::B1_R_Y] = 50 * sin(time); // error
        q[CoordinateName::B1_R_X] = 0;// 50 * sin(time); // error
        q[CoordinateName::F1R_R_X] = 0;//  50 * sin(time);
        q[CoordinateName::F1L_R_X] = 100;
        q[CoordinateName::F2R_R_X] = 100;
        q[CoordinateName::F2L_R_X] = 100;
        q[CoordinateName::F3R_R_X] = 100;
        q[CoordinateName::F3L_R_X] = 100;
        q[CoordinateName::H1R_R_Y] = 100;
        q[CoordinateName::H1L_R_Y] = 100;
        q[CoordinateName::H1R_R_Z] = 100;
        q[CoordinateName::H1L_R_Z] = 100;
        q[CoordinateName::H2R_R_Y] = 100;
        q[CoordinateName::H2L_R_Y] = 100;

        // Task 4.1: draw the skin using wireframe mode
        //*/
        skeletonSkin->bind();
        mat4 maleModelMatrix = mat4(1);
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

        sk->bind();
        mat4 male2ModelMatrix = glm::translate(mat4(), vec3(6.0f, 0.0f, 0.0f));;
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &male2ModelMatrix[0][0]);
        glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
        glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);
        glUniform1i(useSkinningLocation, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        sk->draw();

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