#include "grid.h"

Grid::Grid(const YAML::Node& config) {
    config_.min = glm::vec3(config["box"]["min"][0].as<float>(), config["box"]["min"][1].as<float>(), config["box"]["min"][2].as<float>());
    config_.max = glm::vec3(config["box"]["max"][0].as<float>(), config["box"]["max"][1].as<float>(), config["box"]["max"][2].as<float>());
    config_.resolution = glm::ivec3(config["resolution"][0].as<int>(), config["resolution"][1].as<int>(), config["resolution"][2].as<int>());
    config_.wall_thickness = config["wall_thickness"].as<float>();
    config_.num = config_.resolution.x * config_.resolution.y * config_.resolution.z;

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
    glm::ivec3 q = glm::ivec3(worldToGrid(p));
    q.x = q.x < 0 ? 0 : (q.x >= config_.resolution.x ? config_.resolution.x - 1 : q.x);
    q.y = q.y < 0 ? 0 : (q.y >= config_.resolution.y ? config_.resolution.y - 1 : q.y);
    q.z = q.z < 0 ? 0 : (q.z >= config_.resolution.z ? config_.resolution.z - 1 : q.z);
    return q;
}

glm::vec3 Grid::gridToWorld(const glm::vec3& p) {
    return p * config_.scale + config_.offset;
}