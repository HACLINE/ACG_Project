#include "render.h"
#include <iostream>
#include <GL/glut.h> // Using OpenGL API

Renderer::Renderer(YAML::Node config) {
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

void Renderer::renderMesh(const Mesh& mesh) {
    glBegin(GL_TRIANGLES);
    for (const auto& face : mesh.faces) {
        const glm::vec3& v1 = mesh.vertices[face.v1].position;
        const glm::vec3& v2 = mesh.vertices[face.v2].position;
        const glm::vec3& v3 = mesh.vertices[face.v3].position;

        glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
        glNormal3f(normal.x, normal.y, normal.z);

        glColor3f(0.0f, 0.8f, 0.2f);
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
    glBegin(GL_QUADS);
    glColor3f(0.8f, 0.8f, 0.8f);
    glVertex3f(-100.0f, -10.0f, -100.0f);
    glVertex3f( 100.0f, -10.0f, -100.0f);
    glVertex3f( 100.0f, -10.0f,  100.0f);
    glVertex3f(-100.0f, -10.0f,  100.0f);
    glEnd();
}

void Renderer::swapBuffers() {
    glutSwapBuffers();
}

void Renderer::renderRigidbody(Rigidbody* rigidbody) {
    renderMesh(rigidbody->getCurrentMesh());
}

void Renderer::renderFluid(Fluid* fluid) {
    renderParticles(fluid->getParticles());
}

void Renderer::renderSimulation(const Simulation& simulation) {
    for (int i = 0; i < simulation.getNumRigidbodies(); i++) {
        renderRigidbody(simulation.getRigidbody(i));
    }
    for (int i = 0; i < simulation.getNumFluids(); i++) {
        renderFluid(simulation.getFluid(i));
    }
}
