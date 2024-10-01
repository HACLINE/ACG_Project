#include "rigid_body.h"
#include "geometry.h"
#include <glm/glm.hpp>

#include <iostream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

glm::vec3 Rigidbody::gravity = glm::vec3(0.0f, -9.8f, 0.0f);

Rigidbody::Rigidbody() : velocity_({0.0f, 0.0f, 0.0f}), mass_(1.0f) {}

Rigidbody::~Rigidbody() {}

Rigidbody::Rigidbody(const Mesh& mesh, const glm::vec3 velocity, const glm::vec3 acceleration, const float mass) : mesh_(mesh), velocity_(velocity), acceleration_(acceleration), mass_(mass) {}

Rigidbody::Rigidbody(const std::string& filename, const glm::vec3 velocity, const glm::vec3 acceleration, const float mass) : velocity_(velocity), acceleration_(acceleration), mass_(mass) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                filename.c_str(), (filename + "/").c_str(), true);
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return;
    }

    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];
            glm::i32vec3 face;
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                face[v] = idx.vertex_index;
            }
            mesh_.faces.push_back(Face{face.x, face.y, face.z});

            index_offset += fv;
        }
    }
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        mesh_.vertices.push_back(Vertex{
            glm::vec3(attrib.vertices[i], attrib.vertices[i + 1],
                      attrib.vertices[i + 2])});
    }
    std::cout << "[Load] Loaded " << mesh_.faces.size() << " faces and " << mesh_.vertices.size()
                << " vertices from " << filename << std::endl;

}

void Rigidbody::applyForce(const glm::vec3& force) {
    acceleration_ += force / mass_;
}

void Rigidbody::update(float dt) {
    // apply gravity
    applyForce(gravity * mass_);

    // update velocity and position
    velocity_ += acceleration_ * dt;
    mesh_translation(mesh_, velocity_ * dt);
    acceleration_ = glm::vec3(0.0f);
}