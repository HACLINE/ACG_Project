#include "fluid.h"
#include <fstream>
#include <iostream>

Fluid::Fluid() {}

Fluid::~Fluid() {}

Fluid::Fluid(const std::string& path, const YAML::Node& config) {
    std::ifstream fin;
    std::string filename = path + config["name"].as<std::string>();
    fin.open(filename);
    if (!fin) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    float x, y, z, r;
    while (fin >> x >> y >> z >> r) {
        particles_.push_back(Particle{glm::vec3(x, y, z), r});
    }
    fin.close();
    std::cout << "[Load] Load " << particles_.size() << " particles from " << filename << std::endl;
}

void Fluid::update(float dt) {
    return;
}