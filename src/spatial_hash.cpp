#include "spatial_hash.h"

SpatialHash::SpatialHash(): hash_table_(nullptr), kernel_radius_(0), hash_table_size_(0), particles_(std::vector<Particle>()) {}

SpatialHash::SpatialHash(float kernel_radius, int hash_table_size, const std::vector<Particle>& particles): kernel_radius_(kernel_radius), hash_table_size_(hash_table_size), particles_(particles) {
    hash_table_ = new std::vector<int>[hash_table_size_];
}

SpatialHash::~SpatialHash() {
    delete[] hash_table_;
}

int SpatialHash::hash(const glm::vec3& position) {
    int x = (int)floor(float(position.x) / kernel_radius_);
    int y = (int)floor(float(position.y) / kernel_radius_);
    int z = (int)floor(float(position.z) / kernel_radius_);

    int hash = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    return (hash % hash_table_size_ + hash_table_size_) % hash_table_size_;
}

const std::vector<int>& SpatialHash::getCell(const glm::vec3& position) {
    return hash_table_[hash(position)];
}

void SpatialHash::insert(const glm::vec3& position, int index) {
    hash_table_[hash(position)].push_back(index);
}

void SpatialHash::getNeighbors(const glm::vec3& position, std::vector<int>& neighbors) {
    std::vector<int> tmp;
    glm::vec3 tmp_position;
    neighbors.clear();
    bool in[particles_.size()] = {false};
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                tmp_position = position + glm::vec3(i * kernel_radius_, j * kernel_radius_, k * kernel_radius_);
                tmp = getCell(tmp_position);
                for (int index: tmp) {
                    if (!in[index] && glm::length(particles_[index].position - position) < kernel_radius_) {
                        in[index] = true;
                        neighbors.push_back(index);
                    }
                }
            }
        }
    }
}