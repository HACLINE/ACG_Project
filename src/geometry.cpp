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