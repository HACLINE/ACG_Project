#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <unordered_map>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

#include "geometry.h"


class SpatialHash {
public:
    SpatialHash();
    SpatialHash(float, int, const std::vector<Particle>&);
    ~SpatialHash();

    int hash(const glm::vec3&);
    int hash(const Particle& particle) { return hash(particle.position); }

    const std::vector<int>& getCell(const glm::vec3&);
    const std::vector<int>& getCell(const Particle& particle) { return getCell(particle.position); }

    void insert(const glm::vec3&, int);
    void insert(const Particle& particle, int index) { insert(particle.position, index); }

    void getNeighbors(const glm::vec3&, std::vector<int>&);
    void getNeighbors(const Particle& particle, std::vector<int>& neighbors) { getNeighbors(particle.position, neighbors); }

    void reset() {
        for (int i = 0; i < hash_table_size_; ++i) {
            hash_table_[i].clear();
        }
    }
    void update() {
        reset();
        for (int i = 0; i < particles_.size(); ++i) {
            insert(particles_[i], i);
        }
    }

private:
    float kernel_radius_;
    int hash_table_size_;

    std::vector<int>* hash_table_;
    const std::vector<Particle>& particles_;
};

#endif // SPATIAL_HASH_H