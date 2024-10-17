#include "geometry.h"
#include <iostream>

glm::mat3 cross_matrix(const glm::vec3& v) {
    return glm::mat3(0.0f, -v.z, v.y, v.z, 0.0f, -v.x, -v.y, v.x, 0.0f);
}

glm::mat3 rotation_matrix(const glm::quat& orientation) {
    return glm::mat3_cast(orientation);
}

glm::vec3 transform(const glm::vec3& position, const glm::vec3& translation, const glm::quat& orientation) {
    glm::mat3 rotation = rotation_matrix(orientation);
    return rotation * position + translation;
}

Mesh mesh_translation(Mesh mesh, const glm::vec3& translation, const glm::quat& orientation) {
    for (auto& vertex : mesh.vertices) {
        vertex.position = transform(vertex.position, translation, orientation);
    }
    return mesh;
}

glm::vec3 get_spring_force(const Spring& spring, const glm::vec3& dP, const glm::vec3& dV) {
    float dis = glm::length(dP);
    float f_s = - spring.k_s * (dis - spring.rest_length);
    float f_d = spring.k_d * (glm::dot(dP, dV) / dis);
    glm::vec3 force = (f_s + f_d) * glm::normalize(dP);
    return force;
}