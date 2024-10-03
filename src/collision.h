#ifndef COLLISION_H
#define COLLISION_H

#include "rigid_body.h"
#include "fluid.h"

namespace collision {

    void rigidbody_box_collision(Rigidbody*, const glm::vec3&, const glm::vec3&, float, float);

}

#endif