#ifndef YAML_H
#define YAML_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

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
    YAML::Node fluid_config = YAML::LoadFile(cwd + "/config/" + merged_config["load"]["fluid"]["base"].as<std::string>());
    for (int i = 0; i < merged_config["load"]["fluid"]["cfg"].size(); ++i) {
        YAML::Node tmp = merged_config["load"]["fluid"]["cfg"][i];
        merge_config(tmp, fluid_config);
        merged_config["load"]["fluid"]["cfg"][i] = tmp;
    }
    return merged_config;
}

#endif