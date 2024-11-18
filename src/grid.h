#ifndef GRID_H
#define GRID_H

#include <vector>
#include <yaml-cpp/yaml.h>

#include "geometry.h"
#include "utils.h"

struct GridConfig {
    glm::vec3 min, max;
    glm::vec3 offset, scale;
    glm::ivec3 resolution;
    int num;
    float wall_thickness;
};

class Grid {
public:
    Grid(const YAML::Node&);
    ~Grid();
    glm::vec3 worldToGrid(const glm::vec3&);
    glm::ivec3 worldToGridInt(const glm::vec3&);
    glm::vec3 gridToWorld(const glm::vec3&);

public:
    GridConfig config_;
};

template <class T> 
class VecGrid : public Grid {
public:
    VecGrid(const YAML::Node&);
    ~VecGrid();
    T& vec(const glm::ivec3& p) {return vecs_[p.x + p.y * config_.resolution.x + p.z * config_.resolution.x * config_.resolution.y];}
    T cvec(const glm::ivec3&) const;
public:
    T* vecs_;
};

template <class T> 
VecGrid<T>::VecGrid(const YAML::Node& config): Grid(config) {
    vecs_ = new T[config_.num];
}

template <class T> 
VecGrid<T>::~VecGrid() {
    delete[] vecs_;
}

template <class T> 
T VecGrid<T>::cvec(const glm::ivec3& p) const {
    // if (p.x < 0 || p.x >= config_.resolution.x || p.y < 0 || p.y >= config_.resolution.y || p.z < 0 || p.z >= config_.resolution.z) return T();
    // return vecs_[p.x + p.y * config_.resolution.x + p.z * config_.resolution.x * config_.resolution.y];
    glm::ivec3 q = p;
    q.x = q.x < 0 ? 0 : (q.x >= config_.resolution.x ? config_.resolution.x - 1 : q.x);
    q.y = q.y < 0 ? 0 : (q.y >= config_.resolution.y ? config_.resolution.y - 1 : q.y);
    q.z = q.z < 0 ? 0 : (q.z >= config_.resolution.z ? config_.resolution.z - 1 : q.z);
    return vecs_[q.x + q.y * config_.resolution.x + q.z * config_.resolution.x * config_.resolution.y];
}


#endif