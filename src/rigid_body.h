#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <string>
#include "geometry.h"

class Rigidbody {
public:
    Rigidbody();
    Rigidbody(const Mesh&, const glm::vec3 velocity = {0.0f, 0.0f, 0.0f}, const glm::vec3 acceleration = {0.0f, 0.0f, 0.0f}, const float mass = 1.0f);
    Rigidbody(const std::string& filename, const glm::vec3 velocity = {0.0f, 0.0f, 0.0f}, const glm::vec3 acceleration = {0.0f, 0.0f, 0.0f}, const float mass = 1.0f);
    ~Rigidbody();

    void applyForce(const glm::vec3& force);
    void update(float dt);

    inline const Mesh& getMesh() const { return mesh_; }

    static glm::vec3 gravity;

private:
    Mesh mesh_;
    glm::vec3 velocity_;
    glm::vec3 acceleration_;
    float mass_;
};

#endif