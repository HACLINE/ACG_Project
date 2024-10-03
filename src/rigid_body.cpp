#include "rigid_body.h"
#include "geometry.h"
#include <glm/glm.hpp>

#include <iostream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Rigidbody::~Rigidbody() {}

Rigidbody::Rigidbody(const std::string& path, const YAML::Node& config, const std::string& type) {
    velocity_ = glm::vec3(config["velocity"][0].as<float>(), config["velocity"][1].as<float>(), config["velocity"][2].as<float>());
    acceleration_ = glm::vec3(config["acceleration"][0].as<float>(), config["acceleration"][1].as<float>(), config["acceleration"][2].as<float>());
    position_ = glm::vec3(config["position"][0].as<float>(), config["position"][1].as<float>(), config["position"][2].as<float>());
    angular_acceleration_ = glm::vec3(config["angular_acceleration"][0].as<float>(), config["angular_acceleration"][1].as<float>(), config["angular_acceleration"][2].as<float>());
    angular_velocity_ = glm::vec3(config["angular_velocity"][0].as<float>(), config["angular_velocity"][1].as<float>(), config["angular_velocity"][2].as<float>());
    orientation_ = glm::quat(config["orientation"][0].as<float>(), config["orientation"][1].as<float>(), config["orientation"][2].as<float>(), config["orientation"][3].as<float>());
    orientation_ = glm::normalize(orientation_);
    mass_ = config["mass"].as<float>();

    velocity_buffer_ = glm::vec3(0.0f);
    angular_velocity_buffer_ = glm::vec3(0.0f);

    continuous_collision_ = 0;
    do_collision_ = false;
    type_ = type;

    std::string filename = path + config["name"].as<std::string>();
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
        std::cerr << "[Error] Failed to load/parse .obj file" << std::endl;
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
            glm::vec3(attrib.vertices[i] * config["scale"][0].as<float>(), 
                      attrib.vertices[i + 1] * config["scale"][1].as<float>(),
                      attrib.vertices[i + 2] * config["scale"][2].as<float>())});
    }
    std::cout << "[Load] Load " << mesh_.faces.size() << " faces and " << mesh_.vertices.size()
                << " vertices from " << filename << std::endl;
    initInertia();

}

void Rigidbody::initInertia(void) {
    glm::vec3 centroid = glm::vec3(0.0f);
    glm::mat3 inertia = glm::mat3(0.0f);
    float _m = 0.0f;
    for (auto face : mesh_.faces) {
        glm::vec3 v1 = mesh_.vertices[face.v1].position;
        glm::vec3 v2 = mesh_.vertices[face.v2].position;
        glm::vec3 v3 = mesh_.vertices[face.v3].position;
        float v = glm::determinant(glm::mat3(v1, v2, v3));
        _m += v;
        glm::vec3 v4 = v1 + v2 + v3;
        centroid += v * v4;
        inertia += v * (glm::outerProduct(v1, v1) + glm::outerProduct(v2, v2) + glm::outerProduct(v3, v3) + glm::outerProduct(v4, v4));
    }
    centroid /= (4.0f * _m);
    _m /= 6.0f;
    inertia = inertia / 120.0f - _m * glm::outerProduct(centroid, centroid);
    inertia = glm::mat3(inertia[1][1] + inertia[2][2], inertia[0][1], inertia[0][2],
                        inertia[0][1], inertia[0][0] + inertia[2][2], inertia[1][2],
                        inertia[0][2], inertia[1][2], inertia[0][0] + inertia[1][1]);
    inertia *= mass_ / _m;
    inv_inertia_ = glm::inverse(inertia);
    for (auto& vertex : mesh_.vertices) {
        vertex.position -= centroid;
    }
    // std::cout << "Mass: " << _m << std::endl;
    // std::cout << "Centroid: " << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;
    // std::cout << "Inertia: " << inertia[0][0] << " " << inertia[0][1] << " " << inertia[0][2] << std::endl
    //           << "         " << inertia[1][0] << " " << inertia[1][1] << " " << inertia[1][2] << std::endl
    //           << "         " << inertia[2][0] << " " << inertia[2][1] << " " << inertia[2][2] << std::endl;
}

void Rigidbody::applyForce(const glm::vec3& force) {
    acceleration_ += force / mass_;
}

void ImpulseBasedRigidbody::update(float dt) {
    // update velocity and position
    // std::cout << "position: " << position_.x << " " << position_.y << " " << position_.z << std::endl << "velocity: " << velocity_.x << " " << velocity_.y << " " << velocity_.z << std::endl << "acceleration: " << acceleration_.x << " " << acceleration_.y << " " << acceleration_.z << std::endl << "orientation: " << orientation_.x << " " << orientation_.y << " " << orientation_.z << std::endl << "angular_velocity: " << angular_velocity_.x << " " << angular_velocity_.y << " " << angular_velocity_.z << std::endl << "angular_acceleration: " << angular_acceleration_.x << " " << angular_acceleration_.y << " " << angular_acceleration_.z << std::endl << "velocity_buffer: " << velocity_buffer_.x << " " << velocity_buffer_.y << " " << velocity_buffer_.z << std::endl << "angular_velocity_buffer: " << angular_velocity_buffer_.x << " " << angular_velocity_buffer_.y << " " << angular_velocity_buffer_.z << std::endl;
    velocity_ += velocity_buffer_;
    angular_velocity_ += angular_velocity_buffer_;
    velocity_buffer_ = glm::vec3(0.0f);
    angular_velocity_buffer_ = glm::vec3(0.0f);

    position_ += velocity_ * dt + 0.5f * acceleration_ * dt * dt;
    velocity_ += acceleration_ * dt;
    acceleration_ = glm::vec3(0.0f);
    
    orientation_ += glm::quat(0.0f, angular_velocity_.x, angular_velocity_.y, angular_velocity_.z) * orientation_ * 0.5f * dt;
    orientation_ = glm::normalize(orientation_);
    angular_velocity_ += angular_acceleration_ * dt;
    angular_acceleration_ = glm::vec3(0.0f);

    continuous_collision_ = do_collision_ ? continuous_collision_ + 1 : 0;
    do_collision_ = false;
}

float ImpulseBasedRigidbody::restitution_schedule(float restitution) {
    return restitution / (1.0f + log(continuous_collision_ + 1.0f));
}