#include "grid.h"

Grid::Grid(const YAML::Node& config) {
    config_.min = glm::vec3(config["box"]["min"][0].as<float>(), config["box"]["min"][1].as<float>(), config["box"]["min"][2].as<float>());
    config_.max = glm::vec3(config["box"]["max"][0].as<float>(), config["box"]["max"][1].as<float>(), config["box"]["max"][2].as<float>());
    config_.resolution = glm::ivec3(config["resolution"][0].as<int>(), config["resolution"][1].as<int>(), config["resolution"][2].as<int>());
    config_.wall_thickness = config["wall_thickness"].as<float>();

    config_.offset = config_.min;
    config_.scale = (config_.max - config_.min) / glm::vec3(config_.resolution);
}

Grid::~Grid() {

}

glm::vec3 Grid::worldToGrid(const glm::vec3& p) {
    glm::vec3 p_scaled = (p - config_.offset) / config_.scale;
    // p_scaled.x = p_scaled.x < 0.0f ? 0.0f : (p_scaled.x >= config_.resolution.x ? p_scaled.x - 1e-6 : p_scaled.x);
    // p_scaled.y = p_scaled.y < 0.0f ? 0.0f : (p_scaled.y >= config_.resolution.y ? p_scaled.y - 1e-6 : p_scaled.y);
    // p_scaled.z = p_scaled.z < 0.0f ? 0.0f : (p_scaled.z >= config_.resolution.z ? p_scaled.z - 1e-6 : p_scaled.z);
    return p_scaled;
}

glm::ivec3 Grid::worldToGridInt(const glm::vec3& p) {
    return glm::ivec3(worldToGrid(p));
}

glm::vec3 Grid::gridToWorld(const glm::vec3& p) {
    return p * config_.scale + config_.offset;
}

PICFLIPGrid::PICFLIPGrid(const YAML::Node& config): Grid(config) {
    cells_ = new Cell[config_.resolution.x * config_.resolution.y * config_.resolution.z];
}

PICFLIPGrid::~PICFLIPGrid() {
    delete[] cells_;
}

Cell PICFLIPGrid::ccell(const glm::ivec3& p) const {
    if (p.x < 0 || p.x >= config_.resolution.x || p.y < 0 || p.y >= config_.resolution.y || p.z < 0 || p.z >= config_.resolution.z) return Cell();
    return cells_[p.x + p.y * config_.resolution.x + p.z * config_.resolution.x * config_.resolution.y];
}
