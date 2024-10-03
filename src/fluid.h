#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include "geometry.h"

class Fluid {
public:
    Fluid();
    Fluid(const std::string&, const YAML::Node&, const std::string&);
    ~Fluid();

    void update(float dt);

    inline const std::vector<Particle>& getParticles() const { return particles_; }

private:
    std::vector<Particle> particles_;

    std::string type_;
};

#endif