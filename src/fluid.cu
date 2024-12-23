#ifdef HAS_CUDA
#include "fluid.h"
#include "cuda_utils.h"
#include <cstdio>

// applyAcceleration
void Fluid::applyAccelerationCUDA(const glm::vec3& a) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    DFSPHapplyAccelerationTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, num_particles_, a);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

// #########################################################################
// DFSPHFluid
// #########################################################################

__global__ void DFSPHapplyAccelerationTask(Particle* cuda_particles_, int num_particles, const glm::vec3 a) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_particles_[i].acceleration += a;
}

// kernel
__device__ float DFSPHW(const glm::vec3& r, float kernel_radius) {
    float r_len = glm::length(r) / kernel_radius, k = 8.0f / (M_PI * kernel_radius * kernel_radius * kernel_radius);
    if (r_len < 0.5f) {
        return k * (6.0f * r_len * r_len * r_len - 6.0f * r_len * r_len + 1.0f);
    } else if (r_len < 1.0f) {
        return k * 2.0f * (1.0f - r_len) * (1.0f - r_len) * (1.0f - r_len);
    } else {
        return 0.0f;
    }
}

__device__ glm::vec3 DFSPHgradW(const glm::vec3& r, float kernel_radius) {
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
    DFSPHupdateBufferTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_velocity_buffer_, num_particles_, dt, reflect_clip_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHupdateBufferTask(Particle* cuda_particles_, glm::vec3* cuda_velocity_buffer_, int num_particles, float dt, float reflect_clip) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    glm::vec3 delta_v = cuda_velocity_buffer_[i] + cuda_particles_[i].acceleration * dt;
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
    DFSPHupdatePositionTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, num_particles_, velocity_clip_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHupdatePositionTask(Particle* cuda_particles_, int num_particles, float velocity_clip, float dt) {
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
    DFSPHcomputeDensityTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHcomputeDensityTask(Particle* cuda_particles_, DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho = 0.0f;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho += cuda_particles_[neighbor].mass * DFSPHW(r, kernel_radius);
    }
}

// computeAlpha
void DFSPHFluid::computeAlphaCUDA() {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    DFSPHcomputeAlphaTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHcomputeAlphaTask(Particle* cuda_particles_, DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    float square_sum = 0.0f;
    glm::vec3 sum = glm::vec3(0.0f);
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        glm::vec3 grad = DFSPHgradW(r, kernel_radius);
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
    DFSPHcomputeRhoStarTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, rho0_, dt, p_rho_avg);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
    cudaMemcpy(&rho_avg, p_rho_avg, sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(p_rho_avg);
    return rho_avg / num_particles_;
}

__global__ void DFSPHcomputeRhoStarTask(Particle* cuda_particles_, DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float rho0, float dt, float* p_rho_avg) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho_star = cuda_augmented_particles_[i].rho / rho0;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho_star += dt * cuda_particles_[neighbor].mass * glm::dot(cuda_particles_[i].velocity - cuda_particles_[neighbor].velocity, DFSPHgradW(r, kernel_radius));
    }
    cuda_augmented_particles_[i].rho_star = fmaxf(cuda_augmented_particles_[i].rho_star, 1.0f);
    atomicAdd(p_rho_avg, cuda_augmented_particles_[i].rho_star);
}

// computeKappa
void DFSPHFluid::computeKappaCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    DFSPHcomputeKappaTask<<<num_blocks, cuda_block_size_>>>(cuda_augmented_particles_, num_particles_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHcomputeKappaTask(DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float dt) {
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
    DFSPHcomputeRhoDerivativeTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, p_rho_divergence_avg);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
    cudaMemcpy(&rho_divergence_avg, p_rho_divergence_avg, sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(p_rho_divergence_avg);
    return rho_divergence_avg / num_particles_;
}

__global__ void DFSPHcomputeRhoDerivativeTask(Particle* cuda_particles_, DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float* p_rho_divergence_avg) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].rho_derivative = 0.0f;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_augmented_particles_[i].rho_derivative += cuda_particles_[neighbor].mass * glm::dot(cuda_particles_[i].velocity - cuda_particles_[neighbor].velocity, DFSPHgradW(r, kernel_radius));
    }
    atomicAdd(p_rho_divergence_avg, cuda_augmented_particles_[i].rho_derivative);
}

// computeKappaV
void DFSPHFluid::computeKappaVCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    DFSPHcomputeKappaVTask<<<num_blocks, cuda_block_size_>>>(cuda_augmented_particles_, num_particles_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHcomputeKappaVTask(DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    cuda_augmented_particles_[i].kappa = cuda_augmented_particles_[i].alpha * (cuda_augmented_particles_[i].rho_derivative) / dt;
}

// correctVelocityError
void DFSPHFluid::correctVelocityErrorCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    DFSPHcorrectVelocityErrorTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_augmented_particles_, num_particles_, kernel_radius_, dt);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

__global__ void DFSPHcorrectVelocityErrorTask(Particle* cuda_particles_, DFSPHAugmentedParticle* cuda_augmented_particles_, int num_particles, float kernel_radius, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    for (int j = 0; j < cuda_augmented_particles_[i].num_neighbors; ++j) {
        int neighbor = cuda_augmented_particles_[i].cuda_neighbors[j];
        glm::vec3 r = cuda_particles_[i].position - cuda_particles_[neighbor].position;
        cuda_particles_[i].velocity -= dt * cuda_particles_[neighbor].mass * (cuda_augmented_particles_[i].kappa / cuda_augmented_particles_[i].rho + cuda_augmented_particles_[neighbor].kappa / cuda_augmented_particles_[neighbor].rho) * DFSPHgradW(r, kernel_radius);
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

// #########################################################################
// END OF DFSPHFluid
// #########################################################################

// #########################################################################
// PICFLIPFluid
// #########################################################################

__device__ int PICFLIPindex(const glm::ivec3& pos, GridConfig* config_) {
    return pos.x + pos.y * config_->resolution.x + pos.z * config_->resolution.x * config_->resolution.y;
}

__device__ int PICFLIPcindex(const glm::ivec3& pos, GridConfig* config_) {
    glm::vec3 p = glm::vec3(pos);
    p.x = p.x < 0 ? 0 : (p.x >= config_->resolution.x ? config_->resolution.x : p.x);
    p.y = p.y < 0 ? 0 : (p.y >= config_->resolution.y ? config_->resolution.y : p.y);
    p.z = p.z < 0 ? 0 : (p.z >= config_->resolution.z ? config_->resolution.z : p.z);
    return p.x + p.y * config_->resolution.x + p.z * config_->resolution.x * config_->resolution.y;    
}

__device__ glm::vec3 PICFLIPworldToGrid(const glm::vec3& pos, GridConfig* config_) {
    return (pos - config_->offset) / config_->scale;
}

__device__ glm::vec3 PICFLIPgridToWorld(const glm::vec3& pos, GridConfig* config_) {
    return pos * config_->scale + config_->offset;
}

__device__ glm::ivec3 PICFLIPworldToGridInt(const glm::vec3& pos, GridConfig* config_) {
    glm::ivec3 q = glm::ivec3(PICFLIPworldToGrid(pos, config_));
    q.x = q.x < 0 ? 0 : (q.x >= config_->resolution.x ? config_->resolution.x - 1 : q.x);
    q.y = q.y < 0 ? 0 : (q.y >= config_->resolution.y ? config_->resolution.y - 1 : q.y);
    q.z = q.z < 0 ? 0 : (q.z >= config_->resolution.z ? config_->resolution.z - 1 : q.z);
    return q;
}

__device__ float PICFLIPk(const glm::vec3& v) {
    return ((fabs(v.x) < 1.0f && fabs(v.y) < 1.0f && fabs(v.z) < 1.0f) ? ((1.0f - fabs(v.x)) * (1.0f - fabs(v.y)) * (1.0f - fabs(v.z))) : 0.0f);
}

__device__ float PICFLIPscheduler(float x, float temp, float scale) {
    if (x < 0.0f) return 0.0f;
    float ret;
    // ret = scale / (1.0f + exp(-x / temp));
    ret = temp * x;
    ret = (ret > scale) ? scale : ret;
    return ret;
}

__device__ glm::vec4 PICFLIPinterpolateVel(const glm::vec3& pos, PICFLIPCell* grid, GridConfig* config_) {
    glm::ivec3 cell = glm::ivec3(pos);
    glm::vec3 weight = pos - glm::vec3(cell);
    glm::vec3 weights[2] = {glm::vec3(1.0f) - weight, weight};
    glm::ivec3 offset;
    glm::vec4 result = glm::vec4(0.0f);
    for (offset.x = 0; offset.x < 2; ++offset.x)
    for (offset.y = 0; offset.y < 2; ++offset.y)
    for (offset.z = 0; offset.z < 2; ++offset.z) {
        result += weights[offset.x].x * weights[offset.y].y * weights[offset.z].z * grid[PICFLIPcindex(cell + offset, config_)].vel;
    }
    return result;
}

__device__ glm::vec4 PICFLIPinterpolateOrigVel(const glm::vec3& pos, PICFLIPCell* grid, GridConfig* config_) {
    glm::ivec3 cell = glm::ivec3(pos);
    glm::vec3 weight = pos - glm::vec3(cell);
    glm::vec3 weights[2] = {glm::vec3(1.0f) - weight, weight};
    glm::ivec3 offset;
    glm::vec4 result = glm::vec4(0.0f);
    for (offset.x = 0; offset.x < 2; ++offset.x)
    for (offset.y = 0; offset.y < 2; ++offset.y)
    for (offset.z = 0; offset.z < 2; ++offset.z) {
        result += weights[offset.x].x * weights[offset.y].y * weights[offset.z].z * grid[PICFLIPcindex(cell + offset, config_)].orig_vel;
    }
    return result;
}

__device__ glm::vec3 PICFLIPsampleVelocity(PICFLIPCell* grid, const glm::vec3& pos, GridConfig* config_) {
    glm::vec3 vel;
    vel.x = PICFLIPinterpolateVel(pos - glm::vec3(0.0f, 0.5f, 0.5f), grid, config_).x;
    vel.y = PICFLIPinterpolateVel(pos - glm::vec3(0.5f, 0.0f, 0.5f), grid, config_).y;
    vel.z = PICFLIPinterpolateVel(pos - glm::vec3(0.5f, 0.5f, 0.0f), grid, config_).z;
    return vel;
}

__device__ glm::vec3 PICFLIPsampleOrigVelocity(PICFLIPCell* grid, const glm::vec3& pos, GridConfig* config_) {
    glm::vec3 vel;
    vel.x = PICFLIPinterpolateOrigVel(pos - glm::vec3(0.0f, 0.5f, 0.5f), grid, config_).x;
    vel.y = PICFLIPinterpolateOrigVel(pos - glm::vec3(0.5f, 0.0f, 0.5f), grid, config_).y;
    vel.z = PICFLIPinterpolateOrigVel(pos - glm::vec3(0.5f, 0.5f, 0.0f), grid, config_).z;
    return vel;
}

// transferToGrid
void PICFLIPFluid::transferToGridCUDA() {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPtransferToGridPTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_grid_, weight_inf_, num_particles_, cuda_config_);
    CUDA_SYNC();
    num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPtransferToGridGTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_grid_, weight_inf_, num_particles_, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPtransferToGridPTask(Particle* cuda_particles_, PICFLIPCell* cuda_grid_, float weight_inf_, int num_particles_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;
    glm::ivec3 offset, cell;
    cell = PICFLIPworldToGridInt(cuda_particles_[idx].position, config_);
    atomicAdd(&cuda_grid_[PICFLIPindex(cell, config_)].num, 1);
    for (offset.x = cell.x - 1 > 0 ? cell.x - 1 : 0; offset.x <= cell.x + 1 && offset.x < config_->resolution.x; ++offset.x)
    for (offset.y = cell.y - 1 > 0 ? cell.y - 1 : 0; offset.y <= cell.y + 1 && offset.y < config_->resolution.y; ++offset.y)
    for (offset.z = cell.z - 1 > 0 ? cell.z - 1 : 0; offset.z <= cell.z + 1 && offset.z < config_->resolution.z; ++offset.z) {
        int offset_index = PICFLIPindex(offset, config_);
        glm::vec3 pos = PICFLIPworldToGrid(cuda_particles_[idx].position, config_) - glm::vec3(offset);
        glm::vec4 cur_weight = glm::vec4(
            PICFLIPk(pos - glm::vec3(0.0f, 0.5f, 0.5f)), 
            PICFLIPk(pos - glm::vec3(0.5f, 0.0f, 0.5f)), 
            PICFLIPk(pos - glm::vec3(0.5f, 0.5f, 0.0f)), 
            PICFLIPk(pos - glm::vec3(0.5f, 0.5f, 0.5f))
        );
        atomicAdd(&cuda_grid_[offset_index].vel.x, cur_weight.x * cuda_particles_[idx].velocity.x);
        atomicAdd(&cuda_grid_[offset_index].vel.y, cur_weight.y * cuda_particles_[idx].velocity.y);
        atomicAdd(&cuda_grid_[offset_index].vel.z, cur_weight.z * cuda_particles_[idx].velocity.z);

        atomicAdd(&cuda_grid_[offset_index].orig_vel.x, cur_weight.x);
        atomicAdd(&cuda_grid_[offset_index].orig_vel.y, cur_weight.y);
        atomicAdd(&cuda_grid_[offset_index].orig_vel.z, cur_weight.z);
        atomicAdd(&cuda_grid_[offset_index].orig_vel.w, cur_weight.w);
    }
}

__global__ void PICFLIPtransferToGridGTask(Particle* cuda_particles_, PICFLIPCell* cuda_grid_, float weight_inf_, int num_particles_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    cuda_grid_[idx].vel.x = cuda_grid_[idx].orig_vel.x > weight_inf_ ? cuda_grid_[idx].vel.x / cuda_grid_[idx].orig_vel.x : 0.0f;
    cuda_grid_[idx].vel.y = cuda_grid_[idx].orig_vel.y > weight_inf_ ? cuda_grid_[idx].vel.y / cuda_grid_[idx].orig_vel.y : 0.0f;
    cuda_grid_[idx].vel.z = cuda_grid_[idx].orig_vel.z > weight_inf_ ? cuda_grid_[idx].vel.z / cuda_grid_[idx].orig_vel.z : 0.0f;

    cuda_grid_[idx].vel.w = cuda_grid_[idx].orig_vel.w;
    cuda_grid_[idx].orig_vel = cuda_grid_[idx].vel;
}

// marker
void PICFLIPFluid::markerCUDA() {
    int num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPmarkerTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPmarkerTask(PICFLIPCell* cuda_grid_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    cuda_grid_[idx].marker = bool(cuda_grid_[idx].num > 0);
}

// addForce
void PICFLIPFluid::addForceCUDA(const glm::vec3& a, float dt) {
    int num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPaddForceTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, a, dt, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPaddForceTask(PICFLIPCell* cuda_grid_, const glm::vec3 a, float dt, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    cuda_grid_[idx].vel.x += a.x * dt;
    cuda_grid_[idx].vel.y += a.y * dt;
    cuda_grid_[idx].vel.z += a.z * dt;
    
    glm::ivec3 cell = glm::ivec3(idx % config_->resolution.x, (idx / config_->resolution.x) % config_->resolution.y, idx / config_->resolution.x / config_->resolution.y);

    if (cell.x == 0 || cell.x == config_->resolution.x - 1) cuda_grid_[idx].vel.x = 0.0f;
    if (cell.z == 0 || cell.z == config_->resolution.z - 1) cuda_grid_[idx].vel.z = 0.0f;
    if (cell.x == 0 || cell.x == config_->resolution.x - 1 || cell.z == 0 || cell.z == config_->resolution.z - 1) cuda_grid_[idx].vel.y += a.y * dt;
    if (cell.y == 0) cuda_grid_[idx].vel.y = 0.0f;
    if (cell.y == config_->resolution.y - 1 && cuda_grid_[idx].vel.y > 0.0f) cuda_grid_[idx].vel.y = 0.0f;
}

// divergence
void PICFLIPFluid::divergenceCUDA() {
    int num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPdivergenceTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, particles_per_cell_, scheduler_temperature_, scheduler_scale_, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPdivergenceTask(PICFLIPCell* cuda_grid_, int particles_per_cell_, float scheduler_temperature_, float scheduler_scale_,GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    if (!cuda_grid_[idx].marker) return;
    glm::ivec3 cell = glm::ivec3(idx % config_->resolution.x, (idx / config_->resolution.x) % config_->resolution.y, idx / config_->resolution.x / config_->resolution.y);
    glm::vec4 vel_min = cuda_grid_[idx].vel;
    glm::vec3 vel_max;
    vel_max.x = cuda_grid_[PICFLIPcindex(cell + glm::ivec3(1, 0, 0), config_)].vel.x;
    vel_max.y = cuda_grid_[PICFLIPcindex(cell + glm::ivec3(0, 1, 0), config_)].vel.y;
    vel_max.z = cuda_grid_[PICFLIPcindex(cell + glm::ivec3(0, 0, 1), config_)].vel.z;
    cuda_grid_[idx].divergence = glm::dot(vel_max - glm::vec3(vel_min), glm::vec3(1.0f));
    cuda_grid_[idx].divergence -= PICFLIPscheduler(vel_min.w / particles_per_cell_ - 1.0f, scheduler_temperature_, scheduler_scale_);
}

// jacobi
void PICFLIPFluid::jacobiCUDA(int iter) {
    int num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    for (int i = 0; i < iter; ++i) {
        PICFLIPjacobiTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, cuda_config_);
        CUDA_SYNC();
        PICFLIPswapPressureBuffersTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, cuda_config_);
        CUDA_SYNC();
    }
}

__global__ void PICFLIPjacobiTask(PICFLIPCell* cuda_grid_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    if (!cuda_grid_[idx].marker) return;
    glm::ivec3 cell = glm::ivec3(idx % config_->resolution.x, (idx / config_->resolution.x) % config_->resolution.y, idx / config_->resolution.x / config_->resolution.y);
    cuda_grid_[idx].buffer_pressure = - cuda_grid_[idx].divergence;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell + glm::ivec3(1, 0, 0), config_)].pressure;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell - glm::ivec3(1, 0, 0), config_)].pressure;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell + glm::ivec3(0, 1, 0), config_)].pressure;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell - glm::ivec3(0, 1, 0), config_)].pressure;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell + glm::ivec3(0, 0, 1), config_)].pressure;
    cuda_grid_[idx].buffer_pressure += cuda_grid_[PICFLIPcindex(cell - glm::ivec3(0, 0, 1), config_)].pressure;
    cuda_grid_[idx].buffer_pressure /= 6.0f;
}

__global__ void PICFLIPswapPressureBuffersTask(PICFLIPCell* cuda_grid_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    cuda_grid_[idx].pressure = cuda_grid_[idx].buffer_pressure;
}

// subtractVel
void PICFLIPFluid::subtractVelCUDA() {
    int num_blocks = (config_.num + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPsubtractVelTask<<<num_blocks, cuda_block_size_>>>(cuda_grid_, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPsubtractVelTask(PICFLIPCell* cuda_grid_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= config_->num) return;
    glm::ivec3 cell = glm::ivec3(idx % config_->resolution.x, (idx / config_->resolution.x) % config_->resolution.y, idx / config_->resolution.x / config_->resolution.y);
    float pres_max = cuda_grid_[idx].pressure;
    cuda_grid_[idx].vel.x -= pres_max - cuda_grid_[PICFLIPcindex(cell - glm::ivec3(1, 0, 0), config_)].pressure;
    cuda_grid_[idx].vel.y -= pres_max - cuda_grid_[PICFLIPcindex(cell - glm::ivec3(0, 1, 0), config_)].pressure;
    cuda_grid_[idx].vel.z -= pres_max - cuda_grid_[PICFLIPcindex(cell - glm::ivec3(0, 0, 1), config_)].pressure;
}

// transferToParticles
void PICFLIPFluid::transferToParticlesCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPtransferToParticlesTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_grid_, num_particles_, dt, flipness_, vel_discount_, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPtransferToParticlesTask(Particle* cuda_particles_, PICFLIPCell* cuda_grid_, int num_particles_, float dt, float flipness_, float vel_discount_, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;
    glm::vec3 cell_pos = PICFLIPworldToGrid(cuda_particles_[idx].position, config_);

    glm::vec3 pic_vel = PICFLIPsampleVelocity(cuda_grid_, cell_pos, config_);
    glm::vec3 flip_vel = cuda_particles_[idx].velocity + pic_vel - PICFLIPsampleOrigVelocity(cuda_grid_, cell_pos, config_);

    glm::vec3 new_vel = pic_vel * (1.0f - flipness_) + flip_vel * flipness_;
    new_vel *= vel_discount_;

    glm::vec3 cfl = glm::vec3(1.0f, 1.0f, 1.0f) / (config_->scale * dt);
    new_vel.x = new_vel.x > cfl.x ? cfl.x : (new_vel.x < -cfl.x ? -cfl.x : new_vel.x);
    new_vel.y = new_vel.y > cfl.y ? cfl.y : (new_vel.y < -cfl.y ? -cfl.y : new_vel.y);
    new_vel.z = new_vel.z > cfl.z ? cfl.z : (new_vel.z < -cfl.z ? -cfl.z : new_vel.z);

    cuda_particles_[idx].velocity = new_vel;
}

// advect
void PICFLIPFluid::advectCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    PICFLIPadvectTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_grid_, num_particles_, dt, cuda_config_);
    CUDA_SYNC();
}

__global__ void PICFLIPadvectTask(Particle* cuda_particles_, PICFLIPCell* cuda_grid_, int num_particles_, float dt, GridConfig* config_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;

    glm::vec3 cell_pos = PICFLIPworldToGrid(cuda_particles_[idx].position, config_);
    glm::vec3 grid_vel = PICFLIPsampleVelocity(cuda_grid_, cell_pos, config_);
    cell_pos = PICFLIPworldToGrid(cuda_particles_[idx].position + grid_vel * dt * 0.5f, config_);
    glm::vec3 new_pos = cuda_particles_[idx].position + PICFLIPsampleVelocity(cuda_grid_, cell_pos, config_)* dt;
    cell_pos = PICFLIPworldToGrid(new_pos, config_);

    float wall_thickness = config_->wall_thickness;
    cell_pos.x = cell_pos.x < wall_thickness ? wall_thickness : (cell_pos.x > config_->resolution.x - wall_thickness ? config_->resolution.x - wall_thickness : cell_pos.x);
    cell_pos.y = cell_pos.y < wall_thickness ? wall_thickness : (cell_pos.y > config_->resolution.y - wall_thickness ? config_->resolution.y - wall_thickness : cell_pos.y);
    cell_pos.z = cell_pos.z < wall_thickness ? wall_thickness : (cell_pos.z > config_->resolution.z - wall_thickness ? config_->resolution.z - wall_thickness : cell_pos.z);

    cuda_particles_[idx].position = PICFLIPgridToWorld(cell_pos, config_);
}

// #########################################################################
// END OF PICFLIPFluid
// #########################################################################


#endif