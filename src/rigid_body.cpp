#include "rigid_body.h"

#include <iostream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Rigidbody::Rigidbody() {}

Rigidbody::~Rigidbody() {}

Rigidbody::Rigidbody(const Mesh& mesh) : mesh_(mesh) {}

Rigidbody::Rigidbody(const std::string& filename) {
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