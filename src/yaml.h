#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

YAML::Node yaml_solver(const std::string filename, const std::string cwd) {
    YAML::Node config = YAML::LoadFile(cwd + "/config/" + filename);
    std::string base = config["base"].as<std::string>();
    YAML::Node basic_config = YAML::LoadFile(cwd + "/config/" + base);
    // merge basic_config into config
    for (auto it = basic_config.begin(); it != basic_config.end(); ++it) {
        if (!config[it->first]) {
            config[it->first] = it->second;
        }
    }
    return config;
}