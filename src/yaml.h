#ifndef YAML_H
#define YAML_H

#include <yaml-cpp/yaml.h>
#include <fstream>

void merge_config(YAML::Node&, const YAML::Node&);

void merge_config(YAML::Node& merged_config, const YAML::Node& config) {
    if (!merged_config.IsMap()) {
        merged_config = config;
        return;
    }
    for (auto it = config.begin(); it != config.end(); it++) {
        YAML::Node tmp = merged_config[it->first.as<std::string>()];
        merge_config(tmp, it->second);
        merged_config[it->first.as<std::string>()] = tmp;
    }
}

YAML::Node yaml_solver(const std::string filename, const std::string cwd) {
    YAML::Node config = YAML::LoadFile(cwd + "/config/" + filename);
    std::string base = config["base"].as<std::string>();
    YAML::Node merged_config = YAML::LoadFile(cwd + "/config/" + base);
    merge_config(merged_config, config);
    merged_config["cwd"] = YAML::Node(cwd);
    return merged_config;
}

#endif