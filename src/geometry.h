#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

struct Vertex {
    glm::vec3 position;
};

struct Face {
    int v1, v2, v3;
};

struct Particle {
    glm::vec3 position;
    float radius;
};

struct Mesh{
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
};

glm::mat3 cross_matrix(const glm::vec3&);
glm::mat3 rotation_matrix(const glm::quat&);
glm::vec3 transform(const glm::vec3&, const glm::vec3&, const glm::quat&);
Mesh mesh_translation(Mesh, const glm::vec3&, const glm::quat&);

#endif