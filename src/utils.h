#ifndef UTILS_H
#define UTILS_H

#include <iostream>

#include "geometry.h"

std::ostream& operator<<(std::ostream&, const glm::vec3&);

std::ostream& operator<<(std::ostream&, const glm::ivec3&);

std::ostream& operator<<(std::ostream&, const glm::vec4&);

std::ostream& operator<<(std::ostream&, const Particle&);

void saveParticlesToPLY(const std::vector<Particle>& particles, const std::string& filename);

Mesh loadMeshFromOBJ(const std::string& filename);


#endif