#ifdef HAS_CUDA
#include "fluid.h"
#include "cuda_utils.h"
#include <cstdio>

// applyAcceleration
void Fluid::applyAccelerationCUDA(const glm::vec3& a) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    applyAccelerationTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, num_particles_, a);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void applyAccelerationTask(Particle* cuda_particles_, int num_particles, const glm::vec3 a) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_particles_[i].acceleration += a;
}

// kernel
__device__ float W(const glm::vec3& r, float kernel_radius) {
    float r_len = glm::length(r) / kernel_radius, k = 8.0f / (M_PI * kernel_radius * kernel_radius * kernel_radius);
    if (r_len < 0.5f) {
        return k * (6.0f * r_len * r_len * r_len - 6.0f * r_len * r_len + 1.0f);
    } else if (r_len < 1.0f) {
        return k * 2.0f * (1.0f - r_len) * (1.0f - r_len) * (1.0f - r_len);
    } else {
        return 0.0f;
    }
}

__device__ glm::vec3 gradW(const glm::vec3& r, float kernel_radius) {
    float r_len = glm::length(r) / kernel_radius, k = 8.0f / (M_PI * kernel_radius * kernel_radius * kernel_radius);
    if (r_len < 0.5f) {
        return k * (18.0f * r_len - 12.0f) / kernel_radius / kernel_radius * r;
    } else if (r_len < 1.0f) {
        return - k * 6.0f * (1.0f - r_len) * (1.0f - r_len) / kernel_radius / r_len / kernel_radius * r;
    } else {
        return glm::vec3(0.0f);
    }
}

// updateBuffer
void DFSPHFluid::updateBufferCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    updateBufferTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_velocity_buffer_, num_particles_, dt, reflect_clip_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void updateBufferTask(Particle* cuda_particles_, glm::vec3* cuda_velocity_buffer_, int num_particles, float dt, float reflect_clip) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    glm::vec3 delta_v =cuda_velocity_buffer_[i] + cuda_particles_[i].acceleration * dt;
    if (glm::length(delta_v) > reflect_clip) {
        delta_v = reflect_clip * glm::normalize(delta_v);
    }
    cuda_particles_[i].velocity += delta_v;
    cuda_velocity_buffer_[i] = glm::vec3(0.0f);
    cuda_particles_[i].acceleration = glm::vec3(0.0f);
}

// update position
void DFSPHFluid::updatePositionCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    updatePositionTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, num_particles_, velocity_clip_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void updatePositionTask(Particle* cuda_particles_, int num_particles, float velocity_clip, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    float len = glm::length(cuda_particles_[i].velocity);
    if (len > velocity_clip) {
        cuda_particles_[i].velocity = velocity_clip * cuda_particles_[i].velocity / len;
    }
    cuda_particles_[i].position += cuda_particles_[i].velocity * dt;
}

// computeDensity
void DFSPHFluid::computeDensityCUDA() {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeDensityTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void computeDensityTask(Particle* cuda_particles_, AugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho = 0.0f;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho += cuda_particles_[neighbor].mass * W(r, kernel_radius);
    }
}

// computeAlpha
void DFSPHFluid::computeAlphaCUDA() {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeAlphaTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void computeAlphaTask(Particle* cuda_particles_, AugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    float square_sum = 0.0f;
    glm::vec3 sum = glm::vec3(0.0f);
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        glm::vec3 grad = gradW(r, kernel_radius);
        sum += cuda_particles_[neighbor].mass * grad;
        square_sum += cuda_particles_[neighbor].mass * cuda_particles_[neighbor].mass * glm::dot(grad, grad);
    }
    if (glm::dot(sum, sum) + square_sum > 1e-6f) {
        cuda_augmented_particles_[i].alpha = 1.0f / (glm::dot(sum, sum) + square_sum);
    } else {
        cuda_augmented_particles_[i].alpha = 0.0f;
    }
}

// computeRhoStar
float DFSPHFluid::computeRhoStarCUDA(float dt) {
    float rho_avg = 0.0f, *p_rho_avg;
    cudaMalloc(&p_rho_avg, sizeof(float));
    rho_avg = 0.0f;
    cudaMemcpy(p_rho_avg, &rho_avg, sizeof(float), cudaMemcpyHostToDevice);
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeRhoStarTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, rho0_, dt, p_rho_avg);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
    cudaMemcpy(&rho_avg, p_rho_avg, sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(p_rho_avg);
    return rho_avg / num_particles_;
}

__global__ void computeRhoStarTask(Particle* cuda_particles_, AugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float rho0, float dt, float* p_rho_avg) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho_star = cuda_augmented_particles_[i].rho / rho0;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho_star += dt * cuda_particles_[neighbor].mass * glm::dot(cuda_particles_[i].velocity - cuda_particles_[neighbor].velocity, gradW(r, kernel_radius));
    }
    cuda_augmented_particles_[i].rho_star = fmaxf(cuda_augmented_particles_[i].rho_star, 1.0f);
    atomicAdd(p_rho_avg, cuda_augmented_particles_[i].rho_star);
}

// computeKappa
void DFSPHFluid::computeKappaCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeKappaTask<<<num_blocks, cuda_block_size_>>>(cuda_augmented_particles_, num_particles_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void computeKappaTask(AugmentedParticle* cuda_augmented_particles_, int num_particles, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].kappa = cuda_augmented_particles_[i].alpha * (cuda_augmented_particles_[i].rho_star - 1.0f) / dt / dt;
}

// computeRhoDerivative
float DFSPHFluid::computeRhoDerivativeCUDA(void) {
    float rho_divergence_avg = 0.0f, *p_rho_divergence_avg;
    cudaMalloc(&p_rho_divergence_avg, sizeof(float));
    cudaMemcpy(p_rho_divergence_avg, &rho_divergence_avg, sizeof(float), cudaMemcpyHostToDevice);
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeRhoDerivativeTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, p_rho_divergence_avg);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
    cudaMemcpy(&rho_divergence_avg, p_rho_divergence_avg, sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(p_rho_divergence_avg);
    return rho_divergence_avg / num_particles_;
}

__global__ void computeRhoDerivativeTask(Particle* cuda_particles_, AugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float* p_rho_divergence_avg) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho_derivative = 0.0f;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho_derivative += cuda_particles_[neighbor].mass * glm::dot(cuda_particles_[i].velocity - cuda_particles_[neighbor].velocity, gradW(r, kernel_radius));
    }
    atomicAdd(p_rho_divergence_avg, cuda_augmented_particles_[i].rho_derivative);
}

// computeKappaV
void DFSPHFluid::computeKappaVCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    computeKappaVTask<<<num_blocks, cuda_block_size_>>>(cuda_augmented_particles_, num_particles_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void computeKappaVTask(AugmentedParticle* cuda_augmented_particles_, int num_particles, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].kappa = cuda_augmented_particles_[i].alpha * (cuda_augmented_particles_[i].rho_derivative) / dt;
}

// correctVelocityError
void DFSPHFluid::correctVelocityErrorCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    correctVelocityErrorTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void correctVelocityErrorTask(Particle* cuda_particles_, AugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_particles_[i].velocity -= dt * cuda_particles_[neighbor].mass * (cuda_augmented_particles_[i].kappa / cuda_augmented_particles_[i].rho + cuda_augmented_particles_[neighbor].kappa / cuda_augmented_particles_[neighbor].rho) * gradW(r, kernel_radius);
    }
}

// correctDensityError
void DFSPHFluid::correctDensityErrorCUDA(float dt) {
    float rho_avg = computeRhoStarCUDA(dt);
    for (int iter = 0; ((iter < 2) || (rho_avg / rho0_ > 1 + max_density_error_)) && (iter < max_iter_); ++iter) {
        computeKappaCUDA(dt);
        correctVelocityErrorCUDA(dt);
        rho_avg = computeRhoStarCUDA(dt);
    }
}

// correctDivergenceError
void DFSPHFluid::correctDivergenceErrorCUDA(float dt) {
    float rho_divergence_avg = computeRhoDerivativeCUDA();
    for (int iter = 0; ((iter < 1) || (rho_divergence_avg / rho0_ > max_divergence_error_)) && (iter < max_iter_v_); ++iter) {
        computeKappaVCUDA(dt);
        correctVelocityErrorCUDA(dt);
        rho_divergence_avg = computeRhoDerivativeCUDA();
    }
}
#endif