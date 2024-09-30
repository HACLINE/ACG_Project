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

#endif