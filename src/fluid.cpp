#include "fluid.h"
#include <fstream>
#include <iostream>

Fluid::~Fluid() {}

Fluid::Fluid(const std::string& path, const YAML::Node& config, const std::string& type) {
    type_ = type;
    rho0_ = config["rho"].as<float>();

    if (config["init"].as<std::string>() == "fromfile") {
        std::ifstream fin;
        std::string filename = path + config["name"].as<std::string>();
        fin.open(filename);
        if (!fin) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        float x, y, z, r, m;
        while (fin >> x >> y >> z >> r >> m) {
            particles_.push_back(Particle{glm::vec3(x, y, z), glm::vec3(0.0f), glm::vec3(0.0f), r, m});
        }
        fin.close();
        std::cout << "[Load] Load " << particles_.size() << " particles from " << filename << std::endl;
    } else if (config["init"].as<std::string>() == "cube") {
        float x_gap = config["scale"][0].as<float>() / config["num"][0].as<int>(), 
              y_gap = config["scale"][1].as<float>() / config["num"][1].as<int>(), 
              z_gap = config["scale"][2].as<float>() / config["num"][2].as<int>();
        float radius = config["radius"].as<float>(), mass = config["mass"].as<float>();
        float x_start = - config["scale"][0].as<float>() / 2, 
              y_start = - config["scale"][1].as<float>() / 2, 
              z_start = - config["scale"][2].as<float>() / 2;
        for (int i = 0; i < config["num"][0].as<int>(); ++i) {
            for (int j = 0; j < config["num"][1].as<int>(); ++j) {
                for (int k = 0; k < config["num"][2].as<int>(); ++k) {
                    particles_.push_back(Particle{glm::vec3(x_start + i * x_gap, y_start + j * y_gap, z_start + k * z_gap), glm::vec3(0.0f), glm::vec3(0.0f), radius, mass});
                }
            }
        }
        std::cout << "[Load] Load " << particles_.size() << " particles from config" << std::endl;
    }
    velocity_buffer_.resize(particles_.size(), glm::vec3(0.0f));
}

DFSPHFluid::DFSPHFluid(const std::string& path, const YAML::Node& config, const std::string& type): Fluid(path, config, type), spatial_hash_(config["kernel_radius"].as<float>(), config["hash_table_size"].as<int>(), particles_) {
    kernel_radius_ = config["kernel_radius"].as<float>();
    hash_table_size_ = config["hash_table_size"].as<int>();
    augmented_particles_.resize(particles_.size());
    max_iter_ = config["max_iter"].as<int>();
    max_density_error_ = config["max_density_error"].as<float>();
    max_divergence_error_ = config["max_divergence_error"].as<float>();
    max_iter_v_ = config["max_iter_v"].as<int>();
    
    spatial_hash_.update();
    for (int i = 0; i < particles_.size(); ++i) {
        spatial_hash_.getNeighbors(particles_[i], augmented_particles_[i].neighbors);
    }
    computeDensity();
    computeAlpha();

}

void DFSPHFluid::computeDensity() {
    for (int i = 0; i < particles_.size(); ++i) {
        augmented_particles_[i].rho = 0.0f;
        for (int j = 0; j < augmented_particles_[i].neighbors.size(); ++j) {
            int neighbor = augmented_particles_[i].neighbors[j];
            glm::vec3 r = particles_[i].position - particles_[neighbor].position;
            augmented_particles_[i].rho += particles_[neighbor].mass * W(r);
        }
    }
}

void DFSPHFluid::computeAlpha() {
    for (int i = 0; i < particles_.size(); ++i) {
        float square_sum = 0.0f;
        glm::vec3 sum = glm::vec3(0.0f);
        for (int j = 0; j < augmented_particles_[i].neighbors.size(); ++j) {
            int neighbor = augmented_particles_[i].neighbors[j];
            glm::vec3 r = particles_[i].position - particles_[neighbor].position;
            glm::vec3 grad = gradW(r);
            sum += particles_[neighbor].mass * grad;
            square_sum += particles_[neighbor].mass * particles_[neighbor].mass * glm::dot(grad, grad);
        }
        if (glm::dot(sum, sum) + square_sum > 1e-6f) {
            augmented_particles_[i].alpha = 1.0f / (glm::dot(sum, sum) + square_sum);
            // MODIFIED
        } else {
            augmented_particles_[i].alpha = 0.0f;
        }
    }
}

float DFSPHFluid::W(const glm::vec3& r) {
    float r_len = glm::length(r) / kernel_radius_, k = 8.0f / (M_PI * kernel_radius_ * kernel_radius_ * kernel_radius_);
    if (r_len < 0.5f) {
        return k * (6.0f * r_len * r_len * r_len - 6.0f * r_len * r_len + 1.0f);
    } else if (r_len < 1.0f) {
        return k * 2.0f * (1.0f - r_len) * (1.0f - r_len) * (1.0f - r_len);
    } else {
        return 0.0f;
    }
}

glm::vec3 DFSPHFluid::gradW(const glm::vec3& r) {
    float r_len = glm::length(r) / kernel_radius_, k = 8.0f / (M_PI * kernel_radius_ * kernel_radius_ * kernel_radius_);
    if (r_len < 0.5f) {
        return k * (18.0f * r_len - 12.0f) / kernel_radius_ / kernel_radius_ * r;
    } else if (r_len < 1.0f) {
        return - k * 6.0f * (1.0f - r_len) * (1.0f - r_len) / kernel_radius_ / r_len / kernel_radius_ * r;
    } else {
        return glm::vec3(0.0f);
    }
}


float DFSPHFluid::computeRhoStar(float dt) {
    float rho_avg = 0.0f;
    for (int i = 0; i < particles_.size(); ++i) {
        augmented_particles_[i].rho_star = augmented_particles_[i].rho / rho0_;
        for (int j = 0; j < augmented_particles_[i].neighbors.size(); ++j) {
            int neighbor = augmented_particles_[i].neighbors[j];
            glm::vec3 r =  particles_[i].position - particles_[neighbor].position;
            augmented_particles_[i].rho_star += dt * particles_[neighbor].mass * glm::dot(particles_[i].velocity - particles_[neighbor].velocity, gradW(r));
        }
        augmented_particles_[i].rho_star = std::max(augmented_particles_[i].rho_star, 1.0f);
        rho_avg += augmented_particles_[i].rho_star;
    }
    return rho_avg / particles_.size();
}

void DFSPHFluid::computeKappa(float dt) {
    for (int i = 0; i < particles_.size(); ++i) {
        augmented_particles_[i].kappa = augmented_particles_[i].alpha * (augmented_particles_[i].rho_star - 1.0f) / dt / dt;
    }
}

void DFSPHFluid::correctVelocityError(float dt) {
    for (int i = 0; i < particles_.size(); ++i) {
        for (int j = 0; j < augmented_particles_[i].neighbors.size(); ++j) {
            int neighbor = augmented_particles_[i].neighbors[j];
            glm::vec3 r = particles_[i].position - particles_[neighbor].position;

            particles_[i].velocity -= dt * particles_[neighbor].mass * (augmented_particles_[i].kappa / augmented_particles_[i].rho + augmented_particles_[neighbor].kappa / augmented_particles_[neighbor].rho) * gradW(r);
        }
    }
}

void DFSPHFluid::correctDensityError(float dt) {
    float rho_avg = computeRhoStar(dt);
    for (int iter = 0; ((iter < 2) || (rho_avg / rho0_ > 1 + max_density_error_)) && (iter < max_iter_); ++iter) {
        computeKappa(dt);
        correctVelocityError(dt);
        rho_avg = computeRhoStar(dt);
    }
}

float DFSPHFluid::computeRhoDerivative(float dt) {
    float rho_divergence_avg = 0.0f;
    for (int i = 0; i < particles_.size(); ++i) {
        augmented_particles_[i].rho_derivative = 0.0f;
        for (int j = 0; j < augmented_particles_[i].neighbors.size(); ++j) {
            int neighbor = augmented_particles_[i].neighbors[j];
            glm::vec3 r = particles_[i].position - particles_[neighbor].position;
            augmented_particles_[i].rho_derivative += particles_[neighbor].mass * glm::dot(particles_[i].velocity - particles_[neighbor].velocity, gradW(r));
        }
        rho_divergence_avg += augmented_particles_[i].rho_derivative;
    }
    return rho_divergence_avg / particles_.size();
}

void DFSPHFluid::computeKappaV(float dt) {
    for (int i = 0; i < particles_.size(); ++i) {
        augmented_particles_[i].kappa = augmented_particles_[i].alpha * (augmented_particles_[i].rho_derivative) / dt;
    }
}

void DFSPHFluid::correctDivergenceError(float dt) {
    float rho_divergence_avg = 0.0f;
    for (int iter = 0; ((iter < 1) || (rho_divergence_avg / rho0_ > max_divergence_error_)) && (iter < max_iter_v_); ++iter) {
        rho_divergence_avg = computeRhoDerivative(dt);
        computeKappaV(dt);
        correctVelocityError(dt);
    }
}

void DFSPHFluid::update(float dt) {
    // std::cout << "-------------------------------------------------" << std::endl;for (int i = 0; i < 5; ++i) std::cout << i << " position: " << particles_[i].position.x << " " << particles_[i].position.y << " " << particles_[i].position.z << std::endl <<"  velocity: " << particles_[i].velocity.x << " " << particles_[i].velocity.y << " " << particles_[i].velocity.z << std::endl << "  alpha: " << augmented_particles_[i].alpha << std::endl << "  rho: " << augmented_particles_[i].rho << std::endl << "  kappa: " << augmented_particles_[i].kappa << std::endl << "  rho_star: " << augmented_particles_[i].rho_star << std::endl << "  rho_derivative: " << augmented_particles_[i].rho_derivative << std::endl << "  neighbors: " << augmented_particles_[i].neighbors.size() << std::endl;


    for (int i = 0; i < particles_.size(); ++i) {
        particles_[i].velocity += velocity_buffer_[i];
        velocity_buffer_[i] = glm::vec3(0.0f);
        particles_[i].velocity = particles_[i].velocity + dt * particles_[i].acceleration;
        particles_[i].acceleration = glm::vec3(0.0f);
    }

    correctDensityError(dt);
    for (int i = 0; i < particles_.size(); ++i) {
        particles_[i].position += dt * particles_[i].velocity;
    }
    spatial_hash_.update();
    for (int i = 0; i < particles_.size(); ++i) {
        spatial_hash_.getNeighbors(particles_[i], augmented_particles_[i].neighbors);
    }
    computeDensity();
    computeAlpha();
    correctDivergenceError(dt);
    for (int i = 0; i < particles_.size(); ++i) {
        particles_[i].velocity = particles_[i].velocity;
    }
}
