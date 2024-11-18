#ifdef HAS_CUDA
#include "cloth.h"
#include "cuda_runtime.h"
#include "cuda_utils.h"

// #########################################################################
// Cloth
// #########################################################################

// computeNormals
void Cloth::computeNormalsCUDA() {
    cudaMemset(cuda_vertex_norms_, 0, num_particles_ * sizeof(glm::vec3));
    int num_blocks = (num_faces_ + cuda_block_size_ - 1) / cuda_block_size_;
    ClothcomputeFaceNormalsTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_faces_, cuda_vertex_norms_, cuda_face_norms_, num_particles_, num_faces_);
    CUDA_SYNC();
    num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    ClothnormalizeVertexNormalsTask<<<num_blocks, cuda_block_size_>>>(cuda_vertex_norms_, num_particles_);
    CUDA_SYNC();
}

__global__ void ClothcomputeFaceNormalsTask(Particle* particles, Face* faces, glm::vec3* vertex_norms, glm::vec3* face_norms, int num_particles, int num_faces) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_faces) return;

    face_norms[idx] = glm::normalize(glm::cross(particles[faces[idx].v2].position - particles[faces[idx].v1].position, particles[faces[idx].v3].position - particles[faces[idx].v1].position));
    atomicAdd(&vertex_norms[faces[idx].v1].x, face_norms[idx].x);
    atomicAdd(&vertex_norms[faces[idx].v1].y, face_norms[idx].y);
    atomicAdd(&vertex_norms[faces[idx].v1].z, face_norms[idx].z);
    atomicAdd(&vertex_norms[faces[idx].v2].x, face_norms[idx].x);
    atomicAdd(&vertex_norms[faces[idx].v2].y, face_norms[idx].y);
    atomicAdd(&vertex_norms[faces[idx].v2].z, face_norms[idx].z);
    atomicAdd(&vertex_norms[faces[idx].v3].x, face_norms[idx].x);
    atomicAdd(&vertex_norms[faces[idx].v3].y, face_norms[idx].y);
    atomicAdd(&vertex_norms[faces[idx].v3].z, face_norms[idx].z);
}

__global__ void ClothnormalizeVertexNormalsTask(glm::vec3* vertex_norms, int num_particles) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles) return;
    vertex_norms[idx] = glm::normalize(vertex_norms[idx]);
}

// applyAcceleration
void Cloth::applyAccelerationCUDA(const glm::vec3& a) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    ClothapplyAccelerationTask<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_fixed_, num_particles_, a);
    CUDA_SYNC();
}

__global__ void ClothapplyAccelerationTask(Particle* particles, bool* fixed, int num_particles, glm::vec3 a) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles) return;
    if (fixed[idx]) return;
    particles[idx].acceleration += a;
}


// #########################################################################
// XPBDCloth
// #########################################################################

// solveDistanceConstraint
void XPBDCloth::solveDistanceConstraintCUDA(float dt) {
    int num_blocks = (num_distance_constraints_ + cuda_block_size_ - 1) / cuda_block_size_;
    XPBDsolveDistanceConstraintTask<<<num_blocks, cuda_block_size_>>>(cuda_distance_constraints_, cuda_particles_, num_distance_constraints_, cuda_w_, cuda_delta_p_, dt);
    CUDA_SYNC();
}

__global__ void XPBDsolveDistanceConstraintTask(XPBD_DistanceConstraint* distance_constraints_, Particle* particles_, int num_distance_constraints_, float* cuda_w_, glm::vec3 *cuda_delta_p_, float dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_distance_constraints_) return;

    int p1 = distance_constraints_[idx].p1, p2 = distance_constraints_[idx].p2;
    float rest_length = distance_constraints_[idx].rest_length, k = distance_constraints_[idx].stiffness;
    
    float alpha_tilde = k / dt / dt;
    float w = cuda_w_[p1] + cuda_w_[p2];
    glm::vec3 n = particles_[p1].position - particles_[p2].position;
    float dist = glm::length(n);
    n = glm::normalize(n);
    float C = dist - rest_length;
    float delta_lambda = - C / (w + alpha_tilde);
    atomicAdd(&cuda_delta_p_[p1].x, delta_lambda * cuda_w_[p1] * n.x);
    atomicAdd(&cuda_delta_p_[p1].y, delta_lambda * cuda_w_[p1] * n.y);
    atomicAdd(&cuda_delta_p_[p1].z, delta_lambda * cuda_w_[p1] * n.z);
    atomicAdd(&cuda_delta_p_[p2].x, -delta_lambda * cuda_w_[p2] * n.x);
    atomicAdd(&cuda_delta_p_[p2].y, -delta_lambda * cuda_w_[p2] * n.y);
    atomicAdd(&cuda_delta_p_[p2].z, -delta_lambda * cuda_w_[p2] * n.z);
}

// solveBendingConstraint
void XPBDCloth::solveBendingConstraintCUDA(float dt) {
    
}

// update
void XPBDCloth::updateCUDA(float dt) {
    int num_blocks = (num_particles_ + cuda_block_size_ - 1) / cuda_block_size_;
    for (int iter = 0; iter < iters_; iter++) {
        XPBDupdateTask1<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_delta_p_, cuda_p_, num_particles_, damping_, iters_, dt);
        CUDA_SYNC();
        solveDistanceConstraintCUDA(dt / iters_);
        solveBendingConstraintCUDA(dt / iters_);
        XPBDupdateTask2<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_delta_p_, cuda_p_, num_particles_, damping_, iters_, dt);
        CUDA_SYNC();
    }
    XPBDupdateTask3<<<num_blocks, cuda_block_size_>>>(cuda_particles_, cuda_delta_p_, cuda_p_, num_particles_, damping_, iters_, dt);
}

__global__ void XPBDupdateTask1(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;
    float ds = dt / iters_;
    cuda_p_[idx] = cuda_particles_[idx].position;
    cuda_particles_[idx].position += (1.0f - damping_) * cuda_particles_[idx].velocity * ds + cuda_particles_[idx].acceleration * ds * ds;
}

__global__ void XPBDupdateTask2(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;
    float ds = dt / iters_;
    cuda_particles_[idx].position += cuda_delta_p_[idx];
    cuda_particles_[idx].velocity = (cuda_particles_[idx].position - cuda_p_[idx]) / ds;
    cuda_delta_p_[idx] = glm::vec3(0.0f);
}

__global__ void XPBDupdateTask3(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_particles_) return;
    cuda_particles_[idx].acceleration = glm::vec3(0.0f);
}

#endif 