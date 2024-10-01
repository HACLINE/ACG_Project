#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <string>

#include "geometry.h"

class Fluid {
public:
    Fluid();
    Fluid(const std::vector<Particle>& particles);
    Fluid(const std::string& filename);
    ~Fluid();

    void update(float dt);

    inline const std::vector<Particle>& getParticles() const { return particles_; }

private:
    std::vector<Particle> particles_;
};

#endif