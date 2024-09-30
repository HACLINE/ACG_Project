#include "env.h"
#include <string>
#include <iostream>

Simulation::Simulation() {}

Simulation::Simulation(YAML::Node config) {
    std::string rigidbody_path = config["cwd"].as<std::string>() + config["rigidbody"]["path"].as<std::string>() + "/";
    std::string fluid_path = config["cwd"].as<std::string>() + config["fluid"]["path"].as<std::string>() + "/";
    for (int i = 0; i < config["rigidbody"]["cfg"].size(); ++i) {
        addRigidbody(Rigidbody(rigidbody_path + config["rigidbody"]["cfg"][i][0].as<std::string>()));
    }
    for (int i = 0; i < config["fluid"]["cfg"].size(); ++i) {
        addFluid(Fluid(fluid_path + config["fluid"]["cfg"][i][0].as<std::string>()));
    }
    std::cout<<rigidbodies_.size()<<std::endl;
}

Simulation::~Simulation() {}

void Simulation::addRigidbody(const Rigidbody& rigidbody) {
    rigidbodies_.push_back(rigidbody);
}

void Simulation::addFluid(const Fluid& fluid) {
    fluids_.push_back(fluid);
}

void Simulation::update(float dt) {
    return ;
}

const Rigidbody& Simulation::getRigidbody(int i) const{
    assert(i < rigidbodies_.size());
    return rigidbodies_[i];
}

const Fluid& Simulation::getFluid(int i) const{
    assert(i < fluids_.size());
    return fluids_[i];
}