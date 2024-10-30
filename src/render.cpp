#include "render.h"
#include <iostream>
#ifdef __linux__
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif

#include <utils.h>
#include <marching_cube.h>

Renderer::Renderer(YAML::Node config) : config_(config) {
    if (!config["thread"].as<bool>()) {
        initializeOpenGL(config); 
    } 
}

void Renderer::initializeOpenGL(YAML::Node config) {
    int argc = config["init"]["argc"].as<int>();
    char** argv = new char*[argc];
    for (int i = 0; i < argc; i++) {
        argv[i] = new char[config["init"]["argv"][i].as<std::string>().size() + 1];
        strcpy(argv[i], config["init"]["argv"][i].as<std::string>().c_str());
    }
    glutInit(&argc, argv);
    for (int i = 0; i < argc; i++) {
        delete[] argv[i];
    }
    delete[] argv;
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(config["windowsize"][0].as<int>(), config["windowsize"][1].as<int>());
    glutCreateWindow(config["title"].as<std::string>().c_str());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(config["viewport"][0].as<int>(), config["viewport"][1].as<int>(), config["viewport"][2].as<int>(), config["viewport"][3].as<int>());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(config["perspective"]["fovy"].as<float>(), config["perspective"]["aspect"].as<float>(), config["perspective"]["znear"].as<float>(), config["perspective"]["zfar"].as<float>());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(config["clearcolor"][0].as<float>(), config["clearcolor"][1].as<float>(), config["clearcolor"][2].as<float>(), config["clearcolor"][3].as<float>());
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    gluLookAt(config["camera"]["eye"][0].as<float>(), config["camera"]["eye"][1].as<float>(), config["camera"]["eye"][2].as<float>(), config["camera"]["center"][0].as<float>(), config["camera"]["center"][1].as<float>(), config["camera"]["center"][2].as<float>(), config["camera"]["up"][0].as<float>(), config["camera"]["up"][1].as<float>(), config["camera"]["up"][2].as<float>());

    GLfloat light_position[] = {config["light"]["position"][0].as<float>(), config["light"]["position"][1].as<float>(), config["light"]["position"][2].as<float>(), config["light"]["position"][3].as<float>()};
    GLfloat light_ambient[] = {config["light"]["ambient"][0].as<float>(), config["light"]["ambient"][1].as<float>(), config["light"]["ambient"][2].as<float>(), config["light"]["ambient"][3].as<float>() };
    GLfloat light_diffuse[] = {config["light"]["diffuse"][0].as<float>(), config["light"]["diffuse"][1].as<float>(), config["light"]["diffuse"][2].as<float>(), config["light"]["diffuse"][3].as<float>() };
    GLfloat light_specular[] = {config["light"]["specular"][0].as<float>(), config["light"]["specular"][1].as<float>(), config["light"]["specular"][2].as<float>(), config["light"]["specular"][3].as<float>() };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void Renderer::renderMesh(const Mesh& mesh, glm::vec3 color = glm::vec3(0.0f, 0.8f, 0.2f)) {
    glBegin(GL_TRIANGLES);
    for (const auto& face : mesh.faces) {
        const glm::vec3& v1 = mesh.vertices[face.v1].position;
        const glm::vec3& v2 = mesh.vertices[face.v2].position;
        const glm::vec3& v3 = mesh.vertices[face.v3].position;

        glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
        glNormal3f(normal.x, normal.y, normal.z);

        glColor3f(color.x, color.y, color.z);
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v3.x, v3.y, v3.z);
    }
    glEnd();
}

void Renderer::renderParticles(const std::vector<Particle>& particles) {
    for (const auto& particle : particles) {
        glPushMatrix();
        glTranslatef(particle.position.x, particle.position.y, particle.position.z);
        glColor3f(1.0f, 0.0f, 0.0f);
        glutSolidSphere(particle.radius, 20, 20);
        glPopMatrix();
    }
}

void Renderer::renderFloor() {
    // glBegin(GL_QUADS);
    // glColor3f(0.8f, 0.8f, 0.8f);
    // glVertex3f(-100.0f, -10.0f, -100.0f);
    // glVertex3f( 100.0f, -10.0f, -100.0f);
    // glVertex3f( 100.0f, -10.0f,  100.0f);
    // glVertex3f(-100.0f, -10.0f,  100.0f);
    // glEnd();
}

void Renderer::swapBuffers() {
    glutSwapBuffers();
}

void Renderer::renderRigidbody(Rigidbody* rigidbody) {
    renderMesh(rigidbody->getCurrentMesh());
}

#define SPLASH_SURF 0
#define MARCHING_CUBES 1
#define POINT_CLOUD 2
#define VOXEL 3

void Renderer::renderFluid(Fluid* fluid, int method = SPLASH_SURF) {
    if (method == POINT_CLOUD) {
        renderParticles(fluid->getParticles());
        return ;
    }

    const auto& particles = fluid->getParticles();

    if (method == VOXEL) {
        glm::ivec3 gridResolution(40, 40, 40);
        float gridSpacing = 1.0f / 15.0f;
        std::vector<float> densityField = particlesToDensityField(particles, gridResolution, gridSpacing);
        for (int z = 0; z < gridResolution.z; ++z) {
            for (int y = 0; y < gridResolution.y; ++y) {
                for (int x = 0; x < gridResolution.x; ++x) {
                    int index = x + y * gridResolution.x + z * gridResolution.x * gridResolution.y;
                    if (densityField[index] > 0.0f) {
                        glPushMatrix();
                        glTranslatef(x * gridSpacing - 1.0f, y * gridSpacing - 1.0f, z * gridSpacing - 1.0f);
                        glColor3f(0.0f, 0.0f, 1.0f);
                        glutSolidCube(gridSpacing);
                        glPopMatrix();
                    }
                }
            }
        }
    } else if (method == MARCHING_CUBES) {
        glm::ivec3 gridResolution(40, 40, 40);
        float gridSpacing = 1.0f / 15.0f;
        std::vector<float> densityField = particlesToDensityField(particles, gridResolution, gridSpacing);
        std::vector<Vertex> vertices;
        std::vector<Face> indices;
        MarchingCubes::generateSurface(densityField, gridResolution, gridSpacing, vertices, indices);
        Mesh mesh{vertices, indices};
        mesh_subdivision(mesh);

        renderMesh(mesh);
    } else if (method == SPLASH_SURF) {
        saveParticlesToPLY(particles, "particles.ply");

        std::string command = "splashsurf reconstruct particles.ply -o mesh.obj -q -r=0.03 -l=2.0 -c=0.5 -t=0.6 --subdomain-grid=on --mesh-cleanup=on --mesh-smoothing-weights=on --mesh-smoothing-iters=25 --normals=on --normals-smoothing-iters=10";

        system(command.c_str());

        Mesh mesh = loadMeshFromOBJ("mesh.obj");

        renderMesh(mesh);
    }
}

void Renderer::renderCloth(Cloth* cloth) {
    renderMesh(cloth->getMesh(), glm::vec3(0.8f, 0.0f, 0.0f));
}

void Renderer::renderTriangle(Triangle* triangle) {
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.8f);
    glVertex3f(triangle->v1.x, triangle->v1.y, triangle->v1.z);
    glVertex3f(triangle->v2.x, triangle->v2.y, triangle->v2.z);
    glVertex3f(triangle->v3.x, triangle->v3.y, triangle->v3.z);
    glEnd();
}

void Renderer::renderSphere(Sphere* sphere) {
    glPushMatrix();
    glTranslatef(sphere->center.x, sphere->center.y, sphere->center.z);
    glColor3f(0.0f, 0.0f, 0.6f);
    glutSolidSphere(sphere->radius - 0.01, 20, 20);
    glPopMatrix();
}

void Renderer::renderSimulation(const Simulation& simulation) {
    for (int i = 0; i < simulation.getNumRigidbodies(); i++) {
        renderRigidbody(simulation.getRigidbody(i));
    }
    for (int i = 0; i < simulation.getNumFluids(); i++) {
        renderFluid(simulation.getFluid(i));
    }
    for (int i = 0; i < simulation.getNumCloths(); i++) {
        renderCloth(simulation.getCloth(i));
    }
    for (int i = 0; i < simulation.getNumWalls(); i++) {
        renderTriangle(simulation.getWall(i));
    }
    for (int i = 0; i < simulation.getNumSpheres(); i++) {
        renderSphere(simulation.getSphere(i));
    }
}

void Renderer::renderObject(const RenderObject& object) {
    for (const auto& mesh : object.meshes) {
        renderMesh(mesh);
    }
    for (const auto& particles : object.particles) {
        renderParticles(particles);
    }
}

std::vector<float> Renderer::particlesToDensityField(const std::vector<Particle>& particles, const glm::ivec3& gridResolution, float gridSpacing) {
    std::vector<float> densityField(gridResolution.x * gridResolution.y * gridResolution.z, 0.0f);
    for (const auto& particle : particles) {
        glm::vec3 pos = particle.position + glm::vec3(1.05f, 1.05f, 1.05f);
        glm::ivec3 gridIndex = glm::ivec3(pos / gridSpacing);
        int index = gridIndex.x + gridIndex.y * gridResolution.x + gridIndex.z * gridResolution.x * gridResolution.y;
        if (index < 0 || index >= densityField.size()) {
            std::cerr << "[Render WARNING] Index out of bounds: " << index << std::endl;
            continue;
        }
        densityField[index] += 1.0f;
    }
    return densityField;
}