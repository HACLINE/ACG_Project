#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <variant>

#define STRUCTRAL_SPRING 1
#define SHEAR_SPRING 2
#define BEND_SPRING 3

// ENVIRONMENT GEOMETRIES
struct Triangle {
    glm::vec3 v1, v2, v3;
    float thickness, restitution, friction;
    
    Triangle() = default;
    Triangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, float thickness, float restitution, float friction): v1(v1), v2(v2), v3(v3), thickness(thickness), restitution(restitution), friction(friction) {}
};

struct Sphere {
    glm::vec3 center;
    float radius;

    Sphere() = default;
    Sphere(glm::vec3 center, float radius): center(center), radius(radius) {}
};

// END OF ENVIRONMENT GEOMETRIES

struct Vertex {
    glm::vec3 position;
};

struct Face {
    int v1, v2, v3;
};

struct Particle {
    glm::vec3 position, velocity, acceleration;
    float radius, mass;
};

struct Spring {
    int p1, p2;
    float k_s, k_d;
    float rest_length;
    int type;
};

struct AugmentedParticle {
    float rho = 0.0f, alpha = 0.0f, kappa = 0.0f, rho_star = 0.0f, rho_derivative = 0.0f;
    std::vector<int> neighbors = {};
#ifdef HAS_CUDA
    int* cuda_neighbors = nullptr;
    int num_neighbors = 0;
#endif
};

struct Mesh{
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
};

struct RenderObject {
    std::vector<Mesh> meshes;
    std::vector<std::vector<Particle>> particles;
};

glm::mat3 cross_matrix(const glm::vec3&);
glm::mat3 rotation_matrix(const glm::quat&);
glm::vec3 transform(const glm::vec3&, const glm::vec3&, const glm::quat&);
Mesh mesh_translation(Mesh, const glm::vec3&, const glm::quat&);

glm::vec3 get_spring_force(const Spring&, const glm::vec3&, const glm::vec3&);
#endif