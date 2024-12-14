#ifdef HAS_CUDA

#include "collision.h"
#include "cuda_utils.h"

void collision::coupling_init_CUDA(int max_fluid_size, int max_cloth_size, int neighbourhood) {
    CUDA_CHECK_ERROR(cudaMalloc(&fluid_cloth_neighbourhood, max_fluid_size * sizeof(Neighbourhood)));
    Neighbourhood* h_neighbourhood = new Neighbourhood[max_fluid_size];
    for (int i = 0; i < max_fluid_size; ++i) {
        h_neighbourhood[i].num_neighbors = 0;
        cudaMalloc(&h_neighbourhood[i].cuda_neighbors, neighbourhood * sizeof(int));
    }
    CUDA_CHECK_ERROR(cudaMemcpy(fluid_cloth_neighbourhood, h_neighbourhood, max_fluid_size * sizeof(Neighbourhood), cudaMemcpyHostToDevice));
    delete[] h_neighbourhood;

    CUDA_CHECK_ERROR(cudaMalloc(&delta_cloth_positions, max_cloth_size * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMalloc(&delta_old_cloth_positions, max_cloth_size * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMalloc(&delta_fluid_positions, max_fluid_size * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMalloc(&delta_fluid_velocities, max_fluid_size * sizeof(VecWithInt)));
}

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

void collision::PICFLIP_XPBD_collision_CUDA(PICFLIPFluid* fluid, XPBDCloth* cloth, float dt, YAML::Node& cuda, YAML::Node& coupling) {
    cloth->computeNormalsCUDA();
    Particle* fluid_particles = fluid->getParticlesCUDA();
    Particle* cloth_particles = cloth->getParticlesCUDA();
    glm::vec3* cloth_old_positions = cloth->getOldPositionsCUDA();
    glm::vec3* cloth_normals = cloth->getVertexNormalsCUDA();

    int num_fluid_particles = fluid->getNumParticles();
    int num_cloth_particles = cloth->getNumParticles();

    int block_size = cuda["block_size"].as<int>();
    int num_blocks = (num_fluid_particles + block_size - 1) / block_size;

    float r_c, r_p;
    CUDA_CHECK_ERROR(cudaMemcpy(&r_c, &cloth_particles[0].radius, sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(&r_p, &fluid_particles[0].radius, sizeof(float), cudaMemcpyDeviceToHost));
    float r = r_c + r_p;
    std::vector<Particle> fake;

    SpatialHash hash_table(r, coupling["hash_table_size"].as<int>(), fake);
    hash_table.updateAndGetNeighborsCUDA(fluid_particles, cloth_particles, fluid_cloth_neighbourhood, num_fluid_particles, num_cloth_particles, cuda["block_size"].as<int>(), coupling["neighborhood_size"].as<int>());
    
    // Neighbourhood* tmp = new Neighbourhood[num_fluid_particles];
    // CUDA_CHECK_ERROR(cudaMemcpy(tmp, fluid_cloth_neighbourhood, num_fluid_particles * sizeof(Neighbourhood), cudaMemcpyDeviceToHost));
    // for (int i = 0; i < num_fluid_particles; ++i) {
    //     if (tmp[i].num_neighbors > 0) {
    //         std::cout << tmp[i].num_neighbors << ' ' << i << std::endl;
    //     }
    // }
    // delete[] tmp;

    CUDA_CHECK_ERROR(cudaMemset(delta_cloth_positions, 0, num_cloth_particles * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMemset(delta_old_cloth_positions, 0, num_cloth_particles * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMemset(delta_fluid_positions, 0, num_fluid_particles * sizeof(VecWithInt)));
    CUDA_CHECK_ERROR(cudaMemset(delta_fluid_velocities, 0, num_fluid_particles * sizeof(VecWithInt)));

    // num_blocks = (num_fluid_particles * num_cloth_particles + block_size - 1) / block_size;
    num_blocks = (num_fluid_particles + block_size - 1) / block_size;
    PICFLIP_XPBD_collision_Task1<<<num_blocks, block_size>>>(fluid_particles, cloth_particles, cloth_old_positions, cloth_normals, fluid_cloth_neighbourhood, num_fluid_particles, num_cloth_particles, r, dt, delta_cloth_positions, delta_old_cloth_positions, delta_fluid_positions, delta_fluid_velocities);
    CUDA_SYNC();

    num_blocks = (num_fluid_particles + block_size - 1) / block_size;
    PICFLIP_XPBD_collision_Task2<<<num_blocks, block_size>>>(fluid_particles, cloth_particles, cloth_old_positions, cloth_normals, fluid_cloth_neighbourhood, num_fluid_particles, num_cloth_particles, r, dt, delta_cloth_positions, delta_old_cloth_positions, delta_fluid_positions, delta_fluid_velocities);
    CUDA_SYNC();

    num_blocks = (num_cloth_particles + block_size - 1) / block_size;
    PICFLIP_XPBD_collision_Task3<<<num_blocks, block_size>>>(fluid_particles, cloth_particles, cloth_old_positions, cloth_normals, fluid_cloth_neighbourhood, num_fluid_particles, num_cloth_particles, r, dt, delta_cloth_positions, delta_old_cloth_positions, delta_fluid_positions, delta_fluid_velocities);
    CUDA_SYNC();
}

__device__ float collision::calc_wetting_cuda(float dist, float r) {
    if (dist >= r) return 0.0f;
    return 1.0f - dist / r;
}

__global__ void collision::PICFLIP_XPBD_collision_Task1(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities) {
    int fluid_idx = blockIdx.x * blockDim.x + threadIdx.x;
    // int cloth_idx = fluid_idx % num_cloth_particles;
    // fluid_idx /= num_cloth_particles;
    if (fluid_idx >= num_fluid_particles) return;
    for (int j = 0; j < fluid_cloth_neighbourhood[fluid_idx].num_neighbors; ++j) {
        int cloth_idx = fluid_cloth_neighbourhood[fluid_idx].cuda_neighbors[j];
        glm::vec3 cp = cloth_particles[cloth_idx].position;
        glm::vec3 fp = fluid_particles[fluid_idx].position;
        glm::vec3 dir = cp - fp;
        float dist = glm::length(dir);
        if (dist < r) {
            float wetting = calc_wetting_cuda(dist, r);
            atomicAdd(&cloth_particles[cloth_idx].wetting, wetting);

            glm::vec3 normal = cloth_normals[cloth_idx];
            if (glm::dot(dir, normal) < 0.0f) {
                normal = -normal;
            }
            glm::vec3 v0 = (cp - cloth_old_positions[cloth_idx]) / dt;
            glm::vec3 v1 = fluid_particles[fluid_idx].velocity;
            float w0 = 1.0f / cloth_particles[cloth_idx].mass;
            float w1 = 1.0f / fluid_particles[fluid_idx].mass;
            float iconstraintMass = 1.0f / (w0 + w1);
            glm::vec3 dv = v0 - v1;
            if (glm::dot(dv, normal) < 0.0f) {
                float jn = - glm::dot(dv, normal) * iconstraintMass;
                v0 += normal * (jn * w0);
                v1 -= normal * (jn * w1);
                // fluid_particles[fluid_idx].velocity = v1;
                atomicAdd(&delta_fluid_velocities[fluid_idx].vec.x, v1.x - fluid_particles[fluid_idx].velocity.x);
                atomicAdd(&delta_fluid_velocities[fluid_idx].vec.y, v1.y - fluid_particles[fluid_idx].velocity.y);
                atomicAdd(&delta_fluid_velocities[fluid_idx].vec.z, v1.z - fluid_particles[fluid_idx].velocity.z);
                atomicAdd(&delta_fluid_velocities[fluid_idx].idx, 1);
            }
            float pen = r - glm::dot(cp, normal) + glm::dot(fp, normal);
            pen *= iconstraintMass;
            cp += normal * pen * w0;
            atomicAdd(&delta_cloth_positions[cloth_idx].vec.x, cp.x - cloth_particles[cloth_idx].position.x);
            atomicAdd(&delta_cloth_positions[cloth_idx].vec.y, cp.y - cloth_particles[cloth_idx].position.y);
            atomicAdd(&delta_cloth_positions[cloth_idx].vec.z, cp.z - cloth_particles[cloth_idx].position.z);
            atomicAdd(&delta_cloth_positions[cloth_idx].idx, 1);

            atomicAdd(&delta_old_cloth_positions[cloth_idx].vec.x, cp.x - v0.x * dt - cloth_old_positions[cloth_idx].x);
            atomicAdd(&delta_old_cloth_positions[cloth_idx].vec.y, cp.y - v0.y * dt - cloth_old_positions[cloth_idx].y);
            atomicAdd(&delta_old_cloth_positions[cloth_idx].vec.z, cp.z - v0.z * dt - cloth_old_positions[cloth_idx].z);
            atomicAdd(&delta_old_cloth_positions[cloth_idx].idx, 1);

            atomicAdd(&delta_fluid_positions[fluid_idx].vec.x, -normal.x * pen * w1);
            atomicAdd(&delta_fluid_positions[fluid_idx].vec.y, -normal.y * pen * w1);
            atomicAdd(&delta_fluid_positions[fluid_idx].vec.z, -normal.z * pen * w1);
            atomicAdd(&delta_fluid_positions[fluid_idx].idx, 1);

            // cloth_particles[cloth_idx].position = cp;
            // cloth_old_positions[cloth_idx] = cp - v0 * dt;
            // fluid_particles[fluid_idx].position = fp - normal * pen * w1;
        }
            
    }

}

__global__ void collision::PICFLIP_XPBD_collision_Task2(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities) {
    int fluid_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (fluid_idx >= num_fluid_particles) return;
    if (delta_fluid_positions[fluid_idx].idx > 0) {
        fluid_particles[fluid_idx].position.x += delta_fluid_positions[fluid_idx].vec.x / delta_fluid_positions[fluid_idx].idx;
        fluid_particles[fluid_idx].position.y += delta_fluid_positions[fluid_idx].vec.y / delta_fluid_positions[fluid_idx].idx;
        fluid_particles[fluid_idx].position.z += delta_fluid_positions[fluid_idx].vec.z / delta_fluid_positions[fluid_idx].idx;
    }
    if (delta_fluid_velocities[fluid_idx].idx > 0) {
        fluid_particles[fluid_idx].velocity.x += delta_fluid_velocities[fluid_idx].vec.x / delta_fluid_velocities[fluid_idx].idx;
        fluid_particles[fluid_idx].velocity.y += delta_fluid_velocities[fluid_idx].vec.y / delta_fluid_velocities[fluid_idx].idx;
        fluid_particles[fluid_idx].velocity.z += delta_fluid_velocities[fluid_idx].vec.z / delta_fluid_velocities[fluid_idx].idx;
    }
}

__global__ void collision::PICFLIP_XPBD_collision_Task3(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities) {
    int cloth_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (cloth_idx >= num_cloth_particles) return;
    if (delta_cloth_positions[cloth_idx].idx > 0) {
        cloth_particles[cloth_idx].position.x += delta_cloth_positions[cloth_idx].vec.x / delta_cloth_positions[cloth_idx].idx;
        cloth_particles[cloth_idx].position.y += delta_cloth_positions[cloth_idx].vec.y / delta_cloth_positions[cloth_idx].idx;
        cloth_particles[cloth_idx].position.z += delta_cloth_positions[cloth_idx].vec.z / delta_cloth_positions[cloth_idx].idx;
    }
    if (delta_old_cloth_positions[cloth_idx].idx > 0) {
        cloth_old_positions[cloth_idx].x += delta_old_cloth_positions[cloth_idx].vec.x / delta_old_cloth_positions[cloth_idx].idx;
        cloth_old_positions[cloth_idx].y += delta_old_cloth_positions[cloth_idx].vec.y / delta_old_cloth_positions[cloth_idx].idx;
        cloth_old_positions[cloth_idx].z += delta_old_cloth_positions[cloth_idx].vec.z / delta_old_cloth_positions[cloth_idx].idx;
    }
}

#endif