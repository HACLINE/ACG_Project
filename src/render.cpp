#include "render.h"
#include <GL/glut.h> // Using OpenGL API

void Renderer::renderMesh(const std::vector<Vertex>& vertices, const std::vector<Face>& faces) {
    glBegin(GL_TRIANGLES);
    for (const auto& face : faces) {
        const glm::vec3& v1 = vertices[face.v1].position;
        const glm::vec3& v2 = vertices[face.v2].position;
        const glm::vec3& v3 = vertices[face.v3].position;
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
    glVertex3f(-100.0f, 0.0f, -100.0f);
    glVertex3f( 100.0f, 0.0f, -100.0f);
    glVertex3f( 100.0f, 0.0f,  100.0f);
    glVertex3f(-100.0f, 0.0f,  100.0f);
    glEnd();
}

void Renderer::swapBuffers() {
    glutSwapBuffers();
}