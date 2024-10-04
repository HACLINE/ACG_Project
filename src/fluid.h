#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include "geometry.h"
#include "spatial_hash.h"

class Fluid {
public:
    Fluid(const std::string&, const YAML::Node&, const std::string&);
    ~Fluid();

    virtual void update(float) = 0;

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a) { for (int i = 0; i < particles_.size(); ++i) particles_[i].acceleration += a; }
    void applyGravity(const glm::vec3& g) {applyAcceleration(g);}
    void applyForce(const glm::vec3& f, int i) { particles_[i].acceleration += f / particles_[i].mass; }
    void applyForce(const glm::vec3& f) { for (int i = 0; i < particles_.size(); ++i) applyForce(f, i); }

    std::vector<Particle>& getParticles() { return particles_; }

    void addtoVelocityBuffer(const glm::vec3& v, int i) { velocity_buffer_[i] += v; }

    inline const std::vector<Particle>& getParticles() const { return particles_; }

protected:
    std::vector<Particle> particles_;
    std::vector<glm::vec3> velocity_buffer_;
    float rho0_;

    std::string type_;
};

// Divergence-Free Smoothed Particle Hydrodynamics
class DFSPHFluid : public Fluid {
public:
    DFSPHFluid(const std::string&, const YAML::Node&, const std::string&);
    ~DFSPHFluid() {}

    void update(float dt) override;

private:
    float kernel_radius_, max_density_error_, max_divergence_error_;
    int hash_table_size_, max_iter_, max_iter_v_;
    SpatialHash spatial_hash_;

    struct AugmentedParticle {
        float rho = 0.0f, alpha = 0.0f, kappa = 0.0f, rho_star = 0.0f, rho_derivative = 0.0f;
        std::vector<int> neighbors = {};
    };
    std::vector<AugmentedParticle> augmented_particles_;

    void computeDensity(void);
    void computeAlpha(void);
    float computeRhoStar(float);
    float computeRhoDerivative(float);
    void computeKappa(float);
    void computeKappaV(float);
    void correctDensityError(float);
    void correctDivergenceError(float);
    void correctVelocityError(float);

    float W(const glm::vec3&);
    glm::vec3 gradW(const glm::vec3&);
};
#endif