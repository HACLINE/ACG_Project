#include "env.h"
#include <string>
#include <iostream>
#include "collision.h"

Simulation::Simulation() {}

Simulation::Simulation(YAML::Node load, YAML::Node physics) {
    std::string rigidbody_path = load["cwd"].as<std::string>() + load["rigidbody"]["path"].as<std::string>() + "/";
    std::string fluid_path = load["cwd"].as<std::string>() + load["fluid"]["path"].as<std::string>() + "/";
    for (int i = 0; i < load["rigidbody"]["cfg"].size(); ++i) {
        Rigidbody* rigidbody;
        if (load["rigidbody"]["type"].as<std::string>() == "impulse") {
            rigidbody = new ImpulseBasedRigidbody(rigidbody_path, load["rigidbody"]["cfg"][i], load["rigidbody"]["type"].as<std::string>());
        } else {
            std::cerr << "[Error] Invalid rigidtype" << std::endl;
            exit(1);
        }
        addRigidbody(rigidbody);
    }
    for (int i = 0; i < load["fluid"]["cfg"].size(); ++i) {
        Fluid* fluid;
        if (load["fluid"]["type"].as<std::string>() == "DFSPH") {
            fluid = new DFSPHFluid(fluid_path, load["fluid"]["cfg"][i], load["fluid"]["type"].as<std::string>());
        } else {
            std::cerr << "[Error] Invalid fluidtype" << std::endl;
            exit(1);
        }
        addFluid(fluid);
    }

    gravity_ = glm::vec3(physics["gravity"][0].as<float>(), physics["gravity"][1].as<float>(), physics["gravity"][2].as<float>());
    box_min_ = glm::vec3(physics["box"]["min"][0].as<float>(), physics["box"]["min"][1].as<float>(), physics["box"]["min"][2].as<float>());
    box_max_ = glm::vec3(physics["box"]["max"][0].as<float>(), physics["box"]["max"][1].as<float>(), physics["box"]["max"][2].as<float>());
    restitution_ = physics["restitution"].as<float>();
    friction_ = physics["friction"].as<float>();
}

Simulation::~Simulation() {
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        delete rigidbodies_[i];
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        delete fluids_[i];
    }
}

void Simulation::addRigidbody(Rigidbody* rigidbody) {
    rigidbodies_.push_back(rigidbody);
}

void Simulation::addFluid(Fluid* fluid) {
    fluids_.push_back(fluid);
}

void Simulation::update(float dt) {
    // char x;
    // std::cin >> x;
    // Apply gravity
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i]->applyGravity(gravity_);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        fluids_[i]->applyGravity(gravity_);
    }

    // Collision detection
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        collision::rigidbody_box_collision(rigidbodies_[i], box_min_, box_max_, restitution_, friction_);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        collision::fluid_box_collision(fluids_[i], box_min_, box_max_, restitution_, friction_);
    }

    // Last step
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i]->update(dt);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        fluids_[i]->update(dt);
    }
}

Rigidbody* Simulation::getRigidbody(int i) const{
    assert(i < rigidbodies_.size());
    return rigidbodies_[i];
}

Fluid* Simulation::getFluid(int i) const{
    assert(i < fluids_.size());
    return fluids_[i];
}