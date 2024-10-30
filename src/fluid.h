#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#ifdef HAS_CUDA
#include <cuda_runtime.h>
#endif

#include "geometry.h"
#include "grid.h"
#include "spatial_hash.h"
#include "utils.h"

class Fluid {
public:
    Fluid(const std::string&, const YAML::Node&, const std::string&, const YAML::Node&);
    ~Fluid();

    virtual std::string getType() const { return "basic"; }

    virtual void update(float) = 0;

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a);
#ifdef HAS_CUDA
    void applyAccelerationCUDA(const glm::vec3& a);
#endif
    virtual void applyGravity(const glm::vec3&) = 0;
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
    DFSPHFluid(const std::string&, const YAML::Node&, const std::string&, const YAML::Node&, const YAML::Node&);
    ~DFSPHFluid();
    std::string getType() const override { return "DFSPH"; }

    void applyGravity(const glm::vec3& g) {applyAcceleration(g);}

    void update(float dt) override;

private:
    float kernel_radius_, max_density_error_, max_divergence_error_, velocity_clip_, reflect_clip_;
    int hash_table_size_, max_iter_, max_iter_v_, neighborhood_size_;
    SpatialHash spatial_hash_;


    std::vector<DFSPHAugmentedParticle> augmented_particles_;
#ifdef HAS_CUDA
    DFSPHAugmentedParticle* cuda_augmented_particles_;
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
__global__ void DFSPHapplyAccelerationTask(Particle*, int, const glm::vec3);

__global__ void DFSPHupdateBufferTask(Particle*, glm::vec3*, int, float, float);
__global__ void DFSPHupdatePositionTask(Particle*, int, float, float);

__global__ void DFSPHcomputeDensityTask(Particle*, DFSPHAugmentedParticle*, int, float);
__global__ void DFSPHcomputeAlphaTask(Particle*, DFSPHAugmentedParticle*, int, float);
__global__ void DFSPHcomputeRhoStarTask(Particle*, DFSPHAugmentedParticle*, int, float, float, float, float*);
__global__ void DFSPHcomputeKappaTask(DFSPHAugmentedParticle*, int, float);
__global__ void DFSPHcomputeRhoDerivativeTask(Particle*, DFSPHAugmentedParticle*, int, float, float*);
__global__ void DFSPHcomputeKappaVTask(DFSPHAugmentedParticle*, int, float);

__global__ void DFSPHcorrectVelocityErrorTask(Particle*, DFSPHAugmentedParticle*, int, float, float);

__device__ float DFSPHW(const glm::vec3&, float);
__device__ glm::vec3 DFSPHgradW(const glm::vec3&, float);
#endif

class PICFLIPFluid : public Fluid {
public:
    PICFLIPFluid(const std::string&, const YAML::Node&, const std::string&, const YAML::Node&, const YAML::Node&);
    ~PICFLIPFluid();
    std::string getType() const override { return "PICFLIP"; }

    void update(float dt) override;

    void applyGravity(const glm::vec3& g) {accerleration_ = g;}

private:
    float k(const glm::vec3& v) { return ((fabs(v.x) < 1.0f && fabs(v.y) < 1.0f && fabs(v.z) < 1.0f) ? ((1.0f - fabs(v.x)) * (1.0f - fabs(v.y)) * (1.0f - fabs(v.z))) : 0.0f); }

    void swapVelBuffers(void) {std::swap(vel_grid_, buffer_vel_grid_);}
    void swapPressureBuffers(void) {std::swap(pressure_grid_, buffer_pressure_grid_);}
    template <class T> T interpolate(const VecGrid<T>*, const glm::vec3&);
    template <class T> glm::vec3 sampleVelocity(const VecGrid<T>*, const glm::vec3&);
    float scheduler(float);

    void sortParticles(void);
    void transferToGrid(void);
    void marker(void);
    void addForce(const glm::vec3&, float);
    void divergence(void);
    void jacobi(int);
    void subtractVel(void);
    void transferToParticles(float);
    void advect(float);

private:

    PICFLIPGrid *grid_;
    VecGrid<glm::vec4> *orig_vel_grid_, *vel_grid_, *buffer_vel_grid_;
    VecGrid<float> *divergence_grid_;
    VecGrid<float> *pressure_grid_, *buffer_pressure_grid_;

    glm::vec3 accerleration_;
    float particles_per_cell_, flipness_, scheduler_temperature_, scheduler_scale_, vel_discount_, weight_inf_; 
    int jacobi_iters_;
};
#endif