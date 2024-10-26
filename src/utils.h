#ifndef UTILS_H
#define UTILS_H

#include <iostream>

#include "geometry.h"

extern int usr_name;

std::ostream& operator<<(std::ostream&, const glm::vec3&);

std::ostream& operator<<(std::ostream&, const glm::ivec3&);

std::ostream& operator<<(std::ostream&, const glm::vec4&);

std::ostream& operator<<(std::ostream&, const Particle&);


#endif