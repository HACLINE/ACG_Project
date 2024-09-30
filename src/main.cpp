#include "yaml.h"
#include <yaml-cpp/yaml.h>
#include <GL/glut.h>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include "render.h"
#include "env.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define FPS 60
#define VIDEO_LENGTH 0.5

void saveFrame(const std::string& filename, int width, int height) {
    std::vector<unsigned char> buffer(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename.c_str(), width, height, 3, buffer.data(), width * 3);
}

void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
    GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

/*
./main --config config.yaml
*/
int main(int argc, char* argv[]) {

    // make args into a map
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
        assert(argv[i][0] == '-' && argv[i][1] == '-');
        argv[i] += 2;
        args[argv[i]] = argv[i + 1];
    }

    // load config
    assert(args.find("config") != args.end());
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        perror("getcwd() error");
        return 1;
    }
    YAML::Node config = yaml_solver(args["config"], std::string(cwd));

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("WoBingWang");

    glEnable(GL_DEPTH_TEST);

    glViewport(0, 0, 800, 600);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 800.0 / 600.0, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    setupLighting();

    Simulation simulation;
    std::vector<Vertex> solidVertices = {};
    std::vector<Face> solidFaces = {};
    std::vector<Particle> liquidParticles = {};
    simulation.initialize(solidVertices, solidFaces, liquidParticles);

    Renderer renderer;

    for (int _ = 0; _ < FPS * VIDEO_LENGTH; _++) {
        std::cout << "Generating frame " << _ << std::endl;
        simulation.update(1.0f / FPS);

        std::vector<Vertex> solidMeshVertices;
        std::vector<Face> solidMeshFaces;
        simulation.getSolidMesh(solidMeshVertices, solidMeshFaces);

        std::vector<Particle> updatedParticles;
        simulation.getLiquidParticles(updatedParticles);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderer.renderFloor();
        renderer.renderMesh(solidMeshVertices, solidMeshFaces);
        renderer.renderParticles(updatedParticles);

        std::string file_name = "./figures/frame_" + std::to_string(_) + ".png";
        saveFrame(file_name, 800, 600);

        renderer.swapBuffers();

        GLenum err;
        while ((err = glGetError()) != GL_NO_ERROR) {
            std::cerr << "OpenGL error: " << err << std::endl;
        }
    }

    return 0;
}