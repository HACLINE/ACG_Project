#include "env.h"
#include <string>
#include <iostream>
#include "collision.h"

Simulation::Simulation() {}

Simulation::Simulation(YAML::Node load, YAML::Node physics) {
    std::string rigidbody_path = load["cwd"].as<std::string>() + load["rigidbody"]["path"].as<std::string>() + "/";
    std::string fluid_path = load["cwd"].as<std::string>() + load["fluid"]["path"].as<std::string>() + "/";
    for (int i = 0; i < load["rigidbody"]["cfg"].size(); ++i) {
        addRigidbody(Rigidbody(rigidbody_path, load["rigidbody"]["cfg"][i]));
    }
    for (int i = 0; i < load["fluid"]["cfg"].size(); ++i) {
        addFluid(Fluid(fluid_path, load["fluid"]["cfg"][i]));
    }

    gravity_ = glm::vec3(physics["gravity"][0].as<float>(), physics["gravity"][1].as<float>(), physics["gravity"][2].as<float>());
    box_min_ = glm::vec3(physics["box"]["min"][0].as<float>(), physics["box"]["min"][1].as<float>(), physics["box"]["min"][2].as<float>());
    box_max_ = glm::vec3(physics["box"]["max"][0].as<float>(), physics["box"]["max"][1].as<float>(), physics["box"]["max"][2].as<float>());
    restitution_ = physics["restitution"].as<float>();
    friction_ = physics["friction"].as<float>();
}

Simulation::~Simulation() {}

void Simulation::addRigidbody(const Rigidbody& rigidbody) {
    rigidbodies_.push_back(rigidbody);
}

void Simulation::addFluid(const Fluid& fluid) {
    fluids_.push_back(fluid);
}

void Simulation::update(float dt) {
    // Apply gravity
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i].applyGravity(gravity_);
    }
    // for (int i = 0; i < fluids_.size(); ++i) {
    //     fluids_[i].applyGravity(gravity_);
    // }

    // Collision detection
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        collision::rigidbody_box_collision(rigidbodies_[i], box_min_, box_max_, restitution_, friction_);
    }


    // Last step
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i].update(dt);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        fluids_[i].update(dt);
    }
}

const Rigidbody& Simulation::getRigidbody(int i) const{
    assert(i < rigidbodies_.size());
    return rigidbodies_[i];
}

const Fluid& Simulation::getFluid(int i) const{
    assert(i < fluids_.size());
    return fluids_[i];
}