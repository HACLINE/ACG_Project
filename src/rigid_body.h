#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <string>
#include <yaml-cpp/yaml.h>
#include "geometry.h"

class Rigidbody {
public:
    Rigidbody(const std::string&, const YAML::Node&);
    ~Rigidbody();

    void applyGravity(const glm::vec3& gravity) { acceleration_ += gravity; }
    void applyForce(const glm::vec3& force);

    void setVelocity(const glm::vec3& velocity) { velocity_ = velocity; }
    void setAcceleration(const glm::vec3& acceleration) { acceleration_ = acceleration; }
    void setPosition(const glm::vec3& position) { position_ = position; }
    void setAngularVelocity(const glm::vec3& angular_velocity) { angular_velocity_ = angular_velocity; }
    void setAngularAcceleration(const glm::vec3& angular_acceleration) { angular_acceleration_ = angular_acceleration; }
    void setOrientation(const glm::vec3& orientation) { orientation_ = orientation; }
    void setInvInertia(const glm::mat3& inv_inertia) { inv_inertia_ = inv_inertia; }

    const Mesh& getMesh() const { return mesh_; }
    const Mesh getCurrentMesh() const { return mesh_translation(mesh_, position_, orientation_); }
    glm::mat3 getCurrentInvInertia() const { return rotation_matrix(orientation_) * inv_inertia_ * glm::transpose(rotation_matrix(orientation_)); }
    glm::vec3 getPosition() const { return position_; }
    glm::vec3 getVelocity() const { return velocity_; }
    glm::vec3 getAcceleration() const { return acceleration_; }
    glm::vec3 getAngularVelocity() const { return angular_velocity_; }
    glm::vec3 getAngularAcceleration() const { return angular_acceleration_; }
    glm::quat getOrientation() const { return orientation_; }
    glm::mat3 getInvInertia() const { return inv_inertia_; }
    float getMass() const { return mass_; }

    void addtoVelocityBuffer(const glm::vec3& velocity) { velocity_buffer_ += velocity; }
    void addtoAngularVelocityBuffer(const glm::vec3& angular_velocity) { angular_velocity_buffer_ += angular_velocity; }
    void doCollision() { do_collision_ = true; }

    void update(float dt);
    void resetContinuousCollision() { continuous_collision_ = 0; }

    float restitution_schedule(float);

private:
    Mesh mesh_;
    glm::vec3 velocity_, acceleration_, position_;
    glm::vec3 angular_velocity_, angular_acceleration_;
    glm::quat orientation_;
    glm::mat3 inv_inertia_;
    float mass_;

    glm::vec3 velocity_buffer_, angular_velocity_buffer_;

    int continuous_collision_;
    bool do_collision_;

    void initInertia(void);
};

#endif