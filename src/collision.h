#ifndef COLLISION_H
#define COLLISION_H

#include "rigid_body.h"
#include "fluid.h"
#include "cloth.h"

namespace collision {
    void rigidbody_box_collision(Rigidbody*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
    void fluid_box_collision(Fluid*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
    void fluid_cloth_collision(Fluid*, Cloth*, float);
#ifdef HAS_CUDA
    void fluid_box_collision_CUDA(Fluid*, const glm::vec3&, const glm::vec3&, float, float, const YAML::Node&);
#endif

#ifdef HAS_CUDA
    __global__ void fluid_box_collision_Task(Particle*, glm::vec3*, glm::vec3*, float*, const int, const float, const float);
#endif  
}

#endif