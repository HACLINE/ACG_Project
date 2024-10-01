#include "fluid.h"
#include <fstream>
#include <iostream>

Fluid::Fluid() {}

Fluid::Fluid(const std::vector<Particle>& particles) : particles_(particles) {}

Fluid::~Fluid() {}

Fluid::Fluid(const std::string& filename) {
    std::ifstream fin;
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
    std::cout << "[Load] Loaded " << particles_.size() << " particles from " << filename << std::endl;
}

void Fluid::update(float dt) {
    return;
}