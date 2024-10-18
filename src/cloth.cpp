#include "cloth.h"
#include <fstream>
#include <iostream>
#include <chrono>

Cloth::~Cloth() {}

Cloth::Cloth(const std::string& path, const YAML::Node& config) {
    if (config["init"].as<std::string>() == "fromfile") {
        std::ifstream fin;
        std::string filename = path + config["name"].as<std::string>();
        fin.open(filename);
        if (!fin) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        int opt;
        while (fin >> opt) {
            if (opt == 0) { // particle
                float x, y, z, m;
                fin >> x >> y >> z >> m;
                particles_.push_back(Particle{glm::vec3(x, y, z), glm::vec3(0.0f), glm::vec3(0.0f), m});
            }
            else if (opt == 1) { // spring
                float k_s, k_d, rest_length;
                int p1, p2, type;
                fin >> p1 >> p2 >> k_s >> k_d >> rest_length >> type;
                springs_.push_back(Spring{p1, p2, k_s, k_d, rest_length, type});
            }
            else if (opt == 2) { // face
                int p1, p2, p3;
                fin >> p1 >> p2 >> p3;
                faces_.push_back(Face{p1, p2, p3});
            }
        }
        fixed_ = std::vector<bool>(particles_.size(), false);
        fin.close();
        std::cout << "[Load] Cloth: Load " << particles_.size() << " particles, " << springs_.size() << " springs from " << filename << std::endl;
    } else if (config["init"].as<std::string>() == "square") {
        int x_num = config["num"][0].as<int>(), z_num = config["num"][1].as<int>();
        float x_gap = config["scale"][0].as<float>() / (x_num - 1),
              z_gap = config["scale"][1].as<float>() / (z_num - 1);
        float slope_gap = sqrt(x_gap * x_gap + z_gap * z_gap);
        float mass = config["mass"].as<float>();
        float x_start = - config["scale"][0].as<float>() / 2.0,
              z_start = - config["scale"][1].as<float>() / 2.0;
        float y_pos = config["height"].as<float>();
        for (int i = 0; i < x_num; ++i) {
            for (int j = 0; j < z_num; ++j) {
                particles_.push_back(Particle{glm::vec3(x_start + i * x_gap, y_pos, z_start + j * z_gap), glm::vec3(0.0f), glm::vec3(0.0f), 0.0f, mass});
            }
        }

        // add structual
        for (int i = 0; i < x_num - 1; ++i) {
            for (int j = 0; j < z_num; ++j) {
                springs_.push_back(Spring{(i * z_num) + j, ((i+1) * z_num) + j, config["ks"][0].as<float>(), config["kd"][0].as<float>(), x_gap, STRUCTRAL_SPRING});
            }
        }
        for (int i = 0; i < x_num; ++i) {
            for (int j = 0; j < z_num - 1; ++j) {
                springs_.push_back(Spring{(i * z_num) + j, (i * z_num) + (j+1), config["ks"][0].as<float>(), config["kd"][0].as<float>(), z_gap, STRUCTRAL_SPRING});
            }
        }

        // add shear
        for (int i = 0; i < x_num - 1; ++i) {
            for (int j = 0; j < z_num - 1; ++j) {
                springs_.push_back(Spring{(i * z_num) + j, ((i+1) * z_num) + (j+1), config["ks"][1].as<float>(), config["kd"][1].as<float>(), slope_gap, SHEAR_SPRING});
                springs_.push_back(Spring{(i * z_num) + (j+1), ((i+1) * z_num) + j, config["ks"][1].as<float>(), config["kd"][1].as<float>(), slope_gap, SHEAR_SPRING});
            }
        }

        // add bend
        for (int i = 0; i < x_num; ++i) {
            for (int j = 0; j < z_num - 2; ++j) {
                springs_.push_back(Spring{(i * z_num) + j, (i * z_num) + (j+2), config["ks"][2].as<float>(), config["kd"][2].as<float>(), z_gap + z_gap, BEND_SPRING});
            }
            // springs_.push_back(Spring{(i * z_num) + (z_num-3), (i * z_num) + (z_num-1), config["ks"][2].as<float>(), config["kd"][2].as<float>(), z_gap + z_gap, BEND_SPRING});
        }
        for (int j = 0; j < z_num; ++j) {
            for (int i = 0; i < x_num - 2; ++i) {
                springs_.push_back(Spring{(i * z_num) + j, ((i+2) * z_num) + j, config["ks"][2].as<float>(), config["kd"][2].as<float>(), x_gap + x_gap, BEND_SPRING});
            }
            // springs_.push_back(Spring{((x_num-3) * z_num) + j, ((x_num-1) * z_num) + j, config["ks"][2].as<float>(), config["kd"][2].as<float>(), x_gap + x_gap, BEND_SPRING});
        }

        // init faces
        for (int i = 0; i < x_num - 1; ++i) {
            for (int j = 0; j < z_num - 1; ++j) {
                faces_.push_back(Face{((i+1) * z_num) + j, (i * z_num) + j, ((i+1) * z_num) + (j+1)});
                faces_.push_back(Face{(i * z_num) + (j+1), ((i+1) * z_num) + (j+1), (i * z_num) + j});
            }
        }

        fixed_ = std::vector<bool>(particles_.size(), false);
        // for (int i = 0; i < z_num; ++i) fixed_[i] = fixed_[(x_num-1) * z_num + i] = true;
        // for (int i = 0; i < x_num; ++i) fixed_[i * z_num] = fixed_[i * z_num + (z_num-1)] = true;
        fixed_[0] = fixed_[z_num - 1] = fixed_[particles_.size() - 1] = fixed_[particles_.size() - z_num] = true;

        std::cout << "[Load] Cloth: Load " << particles_.size() << " particles and " << springs_.size() << " springs" << std::endl;
    }

    num_particles_ = particles_.size();
    num_springs_ = springs_.size();
    num_faces_ = faces_.size();
    damping_ = config["damping"].as<float>();
}

void Cloth::setFix(int ind, bool fixed) {
    assert(num_particles_ > ind && ind >= 0);
    fixed_[ind] = fixed;
}

void Cloth::applyForce(const glm::vec3& f, int i) {
    if (!fixed_[i]) {
        particles_[i].acceleration += f / particles_[i].mass;
    }
}

void Cloth::applyDamping() {
    for (int i = 0; i < num_particles_; ++i) {
        applyForce(damping_ * particles_[i].velocity, i);
    }
}

void Cloth::update(float dt) {
    computeForces();

    // Euler Integration
    for (int i = 0; i < num_particles_; ++i) {
        if (!fixed_[i]) {
            particles_[i].velocity += particles_[i].acceleration * dt;
            particles_[i].position += particles_[i].velocity * dt;
        }
        particles_[i].acceleration = glm::vec3(0.0f);
    }

    // provotInverse();
}

void Cloth::computeForces() {
    applyDamping();

    for (int i = 0; i < num_springs_; ++i) {
        glm::vec3 p1 = particles_[springs_[i].p1].position, 
                  p2 = particles_[springs_[i].p2].position;
        glm::vec3 v1 = particles_[springs_[i].p1].velocity, 
                  v2 = particles_[springs_[i].p2].velocity;
        glm::vec3 spring_force = get_spring_force(springs_[i], p1 - p2, v1 - v2);

        applyForce(spring_force, springs_[i].p1);
        applyForce(-spring_force, springs_[i].p2);
    }
}

void Cloth::provotInverse() {
    for (int i = 0; i < num_springs_; ++i) {
        glm::vec3 p1 = particles_[springs_[i].p1].position, 
                    p2 = particles_[springs_[i].p2].position,
                    dp = p1 - p2;
        float dis = glm::length(dp);
        if (dis > springs_[i].rest_length) {
            dis = (dis - springs_[i].rest_length) / 2.0;
            dp = glm::normalize(dp) * dis;
            if (!fixed_[springs_[i].p1]) particles_[springs_[i].p1].velocity -= dp;
            if (!fixed_[springs_[i].p2]) particles_[springs_[i].p2].velocity += dp;
        }
    }
}

Mesh Cloth::getMesh() {
    // std::cout << "getmesh!!!" << std::endl;
    Mesh mesh;
    mesh.vertices.clear();
    mesh.faces = faces_;
    for (int i = 0; i < num_particles_; ++i) {
        mesh.vertices.push_back(Vertex{particles_[i].position});
        // std::cout << particles_[i].position.y << " ";
    }
    // std::cout << std::endl;
    return mesh;
}