#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#ifdef HAS_CUDA
#include <cuda_runtime.h>
#endif

#include "geometry.h"
#include "spatial_hash.h"

class Fluid {
public:
    Fluid(const std::string&, const YAML::Node&, const std::string&, const YAML::Node&);
    ~Fluid();

    virtual void update(float) = 0;

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a);
#ifdef HAS_CUDA
    void applyAccelerationCUDA(const glm::vec3& a);
#endif
    void applyGravity(const glm::vec3& g) {applyAcceleration(g);}
    void applyForce(const glm::vec3& f, int i) { particles_[i].acceleration += f / particles_[i].mass; }
    void applyForce(const glm::vec3& f) { for (int i = 0; i < particles_.size(); ++i) applyForce(f, i); }

    std::vector<Particle>& getParticles() { return particles_; }

    void addtoVelocityBuffer(const glm::vec3& v, int i) { velocity_buffer_[i] += v; }
#ifdef HAS_CUDA
    glm::vec3* getVelocityBufferCUDA(void) { return cuda_velocity_buffer_; }
    Particle* getParticlesCUDA(void) { return cuda_particles_; }
#endif
    int getNumParticles() { return num_particles_; }

    inline const std::vector<Particle>& getParticles() const { return particles_; }

protected:
    const bool cuda_enabled_;
    int cuda_block_size_;
    const std::string type_;
    int num_particles_;

    std::vector<Particle> particles_;
    std::vector<glm::vec3> velocity_buffer_;
#ifdef HAS_CUDA
    Particle* cuda_particles_;
    glm::vec3* cuda_velocity_buffer_;
#endif
    float rho0_;
};

// Divergence-Free Smoothed Particle Hydrodynamics
class DFSPHFluid : public Fluid {
public:
    DFSPHFluid(const std::string&, const YAML::Node&, const std::string&, const YAML::Node&);
    ~DFSPHFluid();

    void update(float dt) override;

private:
    float kernel_radius_, max_density_error_, max_divergence_error_, velocity_clip_, reflect_clip_;
    int hash_table_size_, max_iter_, max_iter_v_, neighborhood_size_;
    SpatialHash spatial_hash_;


    std::vector<AugmentedParticle> augmented_particles_;
#ifdef HAS_CUDA
    AugmentedParticle* cuda_augmented_particles_;
    void updateBufferCUDA(float);
    void updatePositionCUDA(float);

    void computeDensityCUDA(void);
    void computeAlphaCUDA(void);
    float computeRhoStarCUDA(float);
    void computeKappaCUDA(float);
    float computeRhoDerivativeCUDA(void);
    void computeKappaVCUDA(float);

    void correctVelocityErrorCUDA(float);
    void correctDensityErrorCUDA(float);
    void correctDivergenceErrorCUDA(float);
#endif
    void computeDensity(void);
    void computeAlpha(void);
    float computeRhoStar(float);
    void computeKappa(float);
    float computeRhoDerivative(void);
    void computeKappaV(float);

    void correctVelocityError(float);
    void correctDensityError(float);
    void correctDivergenceError(float);

    float W(const glm::vec3&);
    glm::vec3 gradW(const glm::vec3&);
};

#ifdef HAS_CUDA
__global__ void applyAccelerationTask(Particle*, int, const glm::vec3);

__global__ void updateBufferTask(Particle*, glm::vec3*, int, float, float);
__global__ void updatePositionTask(Particle*, int, float, float);

__global__ void computeDensityTask(Particle*, AugmentedParticle*, int, float);
__global__ void computeAlphaTask(Particle*, AugmentedParticle*, int, float);
__global__ void computeRhoStarTask(Particle*, AugmentedParticle*, int, float, float, float, float*);
__global__ void computeKappaTask(AugmentedParticle*, int, float);
__global__ void computeRhoDerivativeTask(Particle*, AugmentedParticle*, int, float, float*);
__global__ void computeKappaVTask(AugmentedParticle*, int, float);

__global__ void correctVelocityErrorTask(Particle*, AugmentedParticle*, int, float, float);

__device__ float W(const glm::vec3&, float);
__device__ glm::vec3 gradW(const glm::vec3&, float);
#endif

#endif