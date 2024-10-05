#ifdef HAS_CUDA

#include "collision.h"
#include "cuda_utils.h"

__global__ void collision::fluid_box_collision_Task(Particle* cuda_particles, glm::vec3* cuda_velocity_buffer, glm::vec3* normal, float* bias, const int num_particles, const float restitution, const float friction) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    for (int j = 0; j < 6; ++j) {
        if (glm::dot(cuda_particles[i].position, normal[j]) + bias[j] < 0.0f) {
            cuda_particles[i].position += - (glm::dot(cuda_particles[i].position, normal[j]) + bias[j]) * normal[j];
            if (glm::dot(cuda_particles[i].velocity, normal[j]) < 0.0f) {
                glm::vec3 vn = glm::dot(cuda_particles[i].velocity, normal[j]) * normal[j], vt = cuda_particles[i].velocity - vn;
                float a = 1.0f - friction * (1 + restitution) * glm::length(vn) / (glm::length(vt) + 1e-6f);
                cuda_velocity_buffer[i] += -restitution * vn + (a > 0.0f ? friction * (1 + restitution) * glm::length(vn) / glm::length(vt) : 0.0f) * vt - cuda_particles[i].velocity;
            }
        }
    }
}

void collision::fluid_box_collision_CUDA(Fluid* fluid, const glm::vec3& box_min, const glm::vec3& box_max, float restitution, float friction, const YAML::Node& cuda) {
     glm::vec3 normal[6] = {
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(-1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, -1.0f)
    };
    float bias[6] = {
        -box_min.x,
        box_max.x,
        -box_min.y,
        box_max.y,
        -box_min.z,
        box_max.z
    };
    Particle* cuda_particles = fluid->getParticlesCUDA();
    glm::vec3* cuda_velocity_buffer = fluid->getVelocityBufferCUDA();
    glm::vec3* cuda_normal;
    float* cuda_bias;
    CUDA_CHECK_ERROR(cudaMalloc(&cuda_normal, 6 * sizeof(glm::vec3)));
    CUDA_CHECK_ERROR(cudaMalloc(&cuda_bias, 6 * sizeof(float)));
    CUDA_CHECK_ERROR(cudaMemcpy(cuda_normal, normal, 6 * sizeof(glm::vec3), cudaMemcpyHostToDevice));
    CUDA_CHECK_ERROR(cudaMemcpy(cuda_bias, bias, 6 * sizeof(float), cudaMemcpyHostToDevice));
    int num_particles = fluid->getNumParticles();
    int block_size = cuda["block_size"].as<int>();
    int grid_size = (num_particles + block_size - 1) / block_size;
    fluid_box_collision_Task<<<grid_size, block_size>>>(cuda_particles, cuda_velocity_buffer, cuda_normal, cuda_bias, num_particles, restitution, friction);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());
}

#endif