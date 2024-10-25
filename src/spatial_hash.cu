#ifdef HAS_CUDA

#include "spatial_hash.h"
#include "cuda_utils.h"

__global__ void hashTask(Particle* cuda_particles, int* hash_list, int* hash_count, const float kernel_radius, const int hash_table_size, const int num_particles) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    int x = (int)floor(float(cuda_particles[i].position.x) / kernel_radius);
    int y = (int)floor(float(cuda_particles[i].position.y) / kernel_radius);
    int z = (int)floor(float(cuda_particles[i].position.z) / kernel_radius);

    hash_list[i] = (((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) % hash_table_size + hash_table_size) % hash_table_size;
    atomicAdd(&hash_count[hash_list[i]], 1);
}

// __global__ void countNeighborsTask(Particle* cuda_particles, DFSPHAugmentedParticle* cuda_augmented_particles, int** hash_table, const int num_particles, const float kernel_radius, const int hash_table_size, int* hash_count) {
//     int i = blockIdx.x * blockDim.x + threadIdx.x;
//     if (i >= num_particles) return;
//     int x = (int)floor(float(cuda_particles[i].position.x) / kernel_radius);
//     int y = (int)floor(float(cuda_particles[i].position.y) / kernel_radius);
//     int z = (int)floor(float(cuda_particles[i].position.z) / kernel_radius);

//     int cnt = 0;
//     for (int dx = -1; dx <= 1; ++dx) {
//         for (int dy = -1; dy <= 1; ++dy) {
//             for (int dz = -1; dz <= 1; ++dz) {
//                 int hash = (((x + dx) * 73856093) ^ ((y + dy) * 19349663) ^ ((z + dz) * 83492791)) % hash_table_size;
//                 hash = (hash + hash_table_size) % hash_table_size;
//                 for (int j = 0; j < hash_count[hash]; ++j) {
//                     float dist = glm::length(cuda_particles[i].position - cuda_particles[hash_table[hash][j]].position);
//                     if (dist < kernel_radius) ++cnt;
//                 }
//             }
//         }
//     }
//     cuda_augmented_particles[i].num_neighbors = cnt;
    
// }

__global__ void getNeighborsTask(Particle* cuda_particles, DFSPHAugmentedParticle* cuda_augmented_particles, int** hash_table, const int num_particles, const float kernel_radius, const int hash_table_size, int* hash_count, const int neighborhood_size) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_particles) return;
    int x = (int)floor(float(cuda_particles[i].position.x) / kernel_radius);
    int y = (int)floor(float(cuda_particles[i].position.y) / kernel_radius);
    int z = (int)floor(float(cuda_particles[i].position.z) / kernel_radius);

    int cnt = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                int hash = (((x + dx) * 73856093) ^ ((y + dy) * 19349663) ^ ((z + dz) * 83492791)) % hash_table_size;
                hash = (hash + hash_table_size) % hash_table_size;
                for (int j = 0; j < hash_count[hash]; ++j) {
                    int k = hash_table[hash][j];
                    float dist = glm::length(cuda_particles[i].position - cuda_particles[k].position);
                    if (dist < kernel_radius && cnt < neighborhood_size) cuda_augmented_particles[i].cuda_neighbors[cnt++] = k;
                }
            }
        }
    }
    cuda_augmented_particles[i].num_neighbors = cnt;
}

void SpatialHash::updateAndGetNeighborsCUDA(Particle* cuda_particles, DFSPHAugmentedParticle* cuda_augmented_particles, int num_particles, const int cuda_block_size, const int neighborhood_size) {
    int* hash_list = nullptr;
    int* hash_count = nullptr;
    int* cpu_hash_list = new int[num_particles];
    int* cpu_hash_count = new int[hash_table_size_], *cpu_hash_count2 = new int[hash_table_size_];
    int** hash_table = nullptr;
    int** cpu_hash_table = new int*[hash_table_size_];
    int** half_cpu_hash_table = new int*[hash_table_size_];

    CUDA_CHECK_ERROR(cudaMalloc(&hash_list, num_particles * sizeof(int)));
    CUDA_CHECK_ERROR(cudaMalloc(&hash_count, hash_table_size_ * sizeof(int)));
    CUDA_CHECK_ERROR(cudaMemset(hash_count, 0, hash_table_size_ * sizeof(int)));

    int num_blocks = (num_particles + cuda_block_size - 1) / cuda_block_size;
    hashTask<<<num_blocks, cuda_block_size>>>(cuda_particles, hash_list, hash_count, kernel_radius_, hash_table_size_, num_particles);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());

    CUDA_CHECK_ERROR(cudaMemcpy(cpu_hash_list, hash_list, num_particles * sizeof(int), cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(cpu_hash_count, hash_count, hash_table_size_ * sizeof(int), cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMalloc(&hash_table, hash_table_size_ * sizeof(int*)));

    for (int i = 0; i < hash_table_size_; ++i) {
        if (cpu_hash_count[i] == 0) {
            half_cpu_hash_table[i] = nullptr;
            cpu_hash_table[i] = nullptr;
            continue;
        }
        CUDA_CHECK_ERROR(cudaMalloc(&half_cpu_hash_table[i], cpu_hash_count[i] * sizeof(int)));
        cpu_hash_table[i] = new int[cpu_hash_count[i]];
    }
    CUDA_CHECK_ERROR(cudaMemcpy(hash_table, half_cpu_hash_table, hash_table_size_ * sizeof(int*), cudaMemcpyHostToDevice));

    memcpy(cpu_hash_count2, cpu_hash_count, hash_table_size_ * sizeof(int));
    for (int i = 0; i < num_particles; ++i) {
        --cpu_hash_count2[cpu_hash_list[i]];
        cpu_hash_table[cpu_hash_list[i]][cpu_hash_count2[cpu_hash_list[i]]] = i;
    }

    for (int i = 0; i < hash_table_size_; ++i) {
        if (cpu_hash_table[i] != nullptr) {
            CUDA_CHECK_ERROR(cudaMemcpy(half_cpu_hash_table[i], cpu_hash_table[i], cpu_hash_count[i] * sizeof(int), cudaMemcpyHostToDevice));
            delete[] cpu_hash_table[i];
        }
    }


    // countNeighborsTask<<<num_blocks, cuda_block_size>>>(cuda_particles, cuda_augmented_particles, hash_table, num_particles, kernel_radius_, hash_table_size_, hash_count);
    // CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    // CUDA_CHECK_ERROR(cudaGetLastError());

    // DFSPHAugmentedParticle tmp;
    // for (int i = 0; i < num_particles; ++i) {
    //     CUDA_CHECK_ERROR(cudaMemcpy(&tmp, &cuda_augmented_particles[i], sizeof(DFSPHAugmentedParticle), cudaMemcpyDeviceToHost));
    //     if (tmp.cuda_neighbors != nullptr) {
    //         CUDA_CHECK_ERROR(cudaFree(tmp.cuda_neighbors));
    //         tmp.cuda_neighbors = nullptr;
    //     }
    //     if (tmp.num_neighbors > 0) {
    //         CUDA_CHECK_ERROR(cudaMalloc(&tmp.cuda_neighbors, tmp.num_neighbors * sizeof(int)));
    //     }
    //     CUDA_CHECK_ERROR(cudaMemcpy(&cuda_augmented_particles[i], &tmp, sizeof(DFSPHAugmentedParticle), cudaMemcpyHostToDevice));
    // }

    getNeighborsTask<<<num_blocks, cuda_block_size>>>(cuda_particles, cuda_augmented_particles, hash_table, num_particles, kernel_radius_, hash_table_size_, hash_count, neighborhood_size);
    CUDA_CHECK_ERROR(cudaDeviceSynchronize());
    CUDA_CHECK_ERROR(cudaGetLastError());

    CUDA_CHECK_ERROR(cudaFree(hash_list));
    CUDA_CHECK_ERROR(cudaFree(hash_count));
    for (int i = 0; i < hash_table_size_; ++i) {
        if (half_cpu_hash_table[i] != nullptr) {
            CUDA_CHECK_ERROR(cudaFree(half_cpu_hash_table[i]));
        }
    }
    CUDA_CHECK_ERROR(cudaFree(hash_table));

    delete[] cpu_hash_list;
    delete[] cpu_hash_count;
    delete[] cpu_hash_count2;
    delete[] cpu_hash_table;
    delete[] half_cpu_hash_table;
}

#endif