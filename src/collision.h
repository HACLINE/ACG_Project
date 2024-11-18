#ifndef COLLISION_H
#define COLLISION_H

#include "rigid_body.h"
#include "fluid.h"
#include "cloth.h"

namespace collision {
    extern Neighbourhood* fluid_cloth_neighbourhood;
#ifdef HAS_CUDA
    extern VecWithInt *delta_cloth_positions, *delta_old_cloth_positions, *delta_fluid_positions, *delta_fluid_velocities;
#endif

    void rigidbody_box_collision(Rigidbody*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
    void fluid_box_collision(Fluid*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
    void fluid_cloth_collision(Fluid*, Cloth*, float, YAML::Node&, YAML::Node&);
#ifdef HAS_CUDA
    void coupling_init_CUDA(int, int, int);

    void fluid_box_collision_CUDA(Fluid*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
    void PICFLIP_XPBD_collision_CUDA(PICFLIPFluid*, XPBDCloth*, float, YAML::Node&, YAML::Node&);
#endif

#ifdef HAS_CUDA
    __global__ void fluid_box_collision_Task(Particle*, glm::vec3*, glm::vec3*, float*, const int, const float, const float);
    __global__ void PICFLIP_XPBD_collision_Task1(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities);
    __global__ void PICFLIP_XPBD_collision_Task2(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities);
    __global__ void PICFLIP_XPBD_collision_Task3(Particle* fluid_particles, Particle* cloth_particles, glm::vec3* cloth_old_positions, glm::vec3* cloth_normals, Neighbourhood* fluid_cloth_neighbourhood, const int num_fluid_particles, const int num_cloth_particles, const float r, const float dt, VecWithInt* delta_cloth_positions, VecWithInt* delta_old_cloth_positions, VecWithInt* delta_fluid_positions, VecWithInt* delta_fluid_velocities);
#endif  
}

#endif