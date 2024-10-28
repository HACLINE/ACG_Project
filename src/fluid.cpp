#include "fluid.h"

#include <fstream>
#include <iostream>
#include <chrono>

Fluid::~Fluid() {
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaFree(cuda_particles_);
        cudaFree(cuda_velocity_buffer_);
    }
#endif
}

Fluid::Fluid(const std::string& path, const YAML::Node& config, const std::string& type, const YAML::Node& cuda): type_(type), cuda_enabled_(cuda["enabled"].as<bool>()), cuda_block_size_(cuda["block_size"].as<int>()) {
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
    num_particles_ = particles_.size();
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaMalloc(&cuda_particles_, particles_.size() * sizeof(Particle));
        cudaMemcpy(cuda_particles_, particles_.data(), particles_.size() * sizeof(Particle), cudaMemcpyHostToDevice);
        cudaMalloc(&cuda_velocity_buffer_, velocity_buffer_.size() * sizeof(glm::vec3));
    }
#endif
}

void Fluid::applyAcceleration(const glm::vec3& a) { 
#ifdef HAS_CUDA
        if (cuda_enabled_) {
            applyAccelerationCUDA(a);
            return;
        }
#endif
        for (int i = 0; i < particles_.size(); ++i) particles_[i].acceleration += a;
    }

DFSPHFluid::DFSPHFluid(const std::string& path, const YAML::Node& config, const std::string& type, const YAML::Node& physics, const YAML::Node& cuda): Fluid(path, config, type, cuda), spatial_hash_(config["kernel_radius"].as<float>(), config["hash_table_size"].as<int>(), particles_) {
    kernel_radius_ = config["kernel_radius"].as<float>();
    hash_table_size_ = config["hash_table_size"].as<int>();
    augmented_particles_.resize(particles_.size());
    max_iter_ = config["max_iter"].as<int>();
    max_density_error_ = config["max_density_error"].as<float>();
    max_divergence_error_ = config["max_divergence_error"].as<float>();
    max_iter_v_ = config["max_iter_v"].as<int>();
    neighborhood_size_ = config["neighborhood_size"].as<int>();
    velocity_clip_ = config["velocity_clip"].as<float>();
    reflect_clip_ = config["reflect_clip"].as<float>();
    
    spatial_hash_.update();
    for (int i = 0; i < particles_.size(); ++i) {
        spatial_hash_.getNeighbors(particles_[i], augmented_particles_[i].neighbors);
    }
#ifdef HAS_CUDA
    if (cuda["enabled"].as<bool>()) {
        for (int i = 0; i < particles_.size(); ++i) {
            cudaMalloc(&augmented_particles_[i].cuda_neighbors, neighborhood_size_ * sizeof(int));
        }
        cudaMalloc(&cuda_augmented_particles_, augmented_particles_.size() * sizeof(DFSPHAugmentedParticle));
        cudaMemcpy(cuda_augmented_particles_, augmented_particles_.data(), augmented_particles_.size() * sizeof(DFSPHAugmentedParticle), cudaMemcpyHostToDevice);
        spatial_hash_.updateAndGetNeighborsCUDA(cuda_particles_, cuda_augmented_particles_, num_particles_, hash_table_size_, neighborhood_size_);
        computeDensityCUDA();
        computeAlphaCUDA();
    } else {
#endif
        computeDensity();
        computeAlpha();
#ifdef HAS_CUDA
    }
#endif
}

DFSPHFluid::~DFSPHFluid() {
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        for (int i = 0; i < particles_.size(); ++i) {
            cudaFree(augmented_particles_[i].cuda_neighbors);
        }
        cudaFree(cuda_augmented_particles_);
    }
#endif
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

float DFSPHFluid::computeRhoDerivative(void) {
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

void DFSPHFluid::correctDivergenceError(float dt) {
    float rho_divergence_avg = 0.0f;
    for (int iter = 0; ((iter < 1) || (rho_divergence_avg / rho0_ > max_divergence_error_)) && (iter < max_iter_v_); ++iter) {
        rho_divergence_avg = computeRhoDerivative();
        computeKappaV(dt);
        correctVelocityError(dt);
    }
}

void DFSPHFluid::update(float dt) {
    // std::cout << "-------------------------------------------------" << std::endl;for (int i = 100; i < 101; ++i) std::cout << i << " position: " << particles_[i].position.x << " " << particles_[i].position.y << " " << particles_[i].position.z << std::endl <<"  velocity: " << particles_[i].velocity.x << " " << particles_[i].velocity.y << " " << particles_[i].velocity.z << std::endl << "  alpha: " << augmented_particles_[i].alpha << std::endl << "  rho: " << augmented_particles_[i].rho << std::endl << "  kappa: " << augmented_particles_[i].kappa << std::endl << "  rho_star: " << augmented_particles_[i].rho_star << std::endl << "  rho_derivative: " << augmented_particles_[i].rho_derivative << std::endl << "  neighbors: " << augmented_particles_[i].neighbors.size() << std::endl;
#ifdef HAS_CUDA
    if (!cuda_enabled_) {
#endif
        // auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < particles_.size(); ++i) {
            velocity_buffer_[i] += dt * particles_[i].acceleration;
            if (glm::length(velocity_buffer_[i]) > reflect_clip_) {
                velocity_buffer_[i] = reflect_clip_ * glm::normalize(velocity_buffer_[i]);
            }
            particles_[i].velocity += velocity_buffer_[i];
            velocity_buffer_[i] = glm::vec3(0.0f);
            particles_[i].acceleration = glm::vec3(0.0f);
        }
        // auto t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "updateVelocities: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        correctDensityError(dt);
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "correctDensityError: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < particles_.size(); ++i) {
            if (glm::length(particles_[i].velocity) > velocity_clip_) {
                particles_[i].velocity = velocity_clip_ * glm::normalize(particles_[i].velocity);
            }
            particles_[i].position += dt * particles_[i].velocity;
        }
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "updatePositions: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        spatial_hash_.update();
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "spatialHashUpdate: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < particles_.size(); ++i) {
            spatial_hash_.getNeighbors(particles_[i], augmented_particles_[i].neighbors);
        }
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "getNeighbors: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        computeDensity();
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "computeDensity: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        computeAlpha();
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "computeAlpha: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // t1 = std::chrono::high_resolution_clock::now();
        correctDivergenceError(dt);
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "correctDivergenceError: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;
#ifdef HAS_CUDA
    } else {
        // auto t2 = std::chrono::high_resolution_clock::now();
        updateBufferCUDA(dt);
        // auto t1 = std::chrono::high_resolution_clock::now();
        // std::cout << "updateBufferCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t2).count() << std::endl;
        correctDensityErrorCUDA(dt);
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "correctDensityErrorCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;
        updatePositionCUDA(dt);
        // t1 = std::chrono::high_resolution_clock::now();
        // std::cout << "updatePositionCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t2).count() << std::endl;
        spatial_hash_.updateAndGetNeighborsCUDA(cuda_particles_, cuda_augmented_particles_, num_particles_, cuda_block_size_, neighborhood_size_);
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "updateNeighborsCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;
        computeDensityCUDA();
        // t1 = std::chrono::high_resolution_clock::now();
        // std::cout << "computeDensityCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t2).count() << std::endl;
        computeAlphaCUDA();
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "computeAlphaCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;
        correctDivergenceErrorCUDA(dt);
        // t1 = std::chrono::high_resolution_clock::now();
        // std::cout << "correctDivergenceErrorCUDA: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t2).count() << std::endl;
        cudaMemcpy(particles_.data(), cuda_particles_, particles_.size() * sizeof(Particle), cudaMemcpyDeviceToHost);
        // t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "memcpy: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;
    }
#endif
}

PICFLIPFluid::PICFLIPFluid(const std::string& path, const YAML::Node& config, const std::string& type, const YAML::Node& physics, const YAML::Node& cuda): Fluid(path, config, type, cuda) {
    grid_ = new PICFLIPGrid(physics);
    vel_grid_ = new VecGrid<glm::vec4>(physics);
    buffer_vel_grid_ = new VecGrid<glm::vec4>(physics);
    orig_vel_grid_ = new VecGrid<glm::vec4>(physics);
    divergence_grid_ = new VecGrid<float>(physics);
    pressure_grid_ = new VecGrid<float>(physics);
    buffer_pressure_grid_ = new VecGrid<float>(physics);

    particles_per_cell_ = config["particles_per_cell"].as<float>();
    jacobi_iters_ = config["jacobi_iters"].as<int>();
    flipness_ = config["flipness"].as<float>();
    scheduler_scale_ = config["scheduler_scale"].as<float>();
    scheduler_temperature_ = config["scheduler_temperature"].as<float>();
    vel_discount_ = config["vel_discount"].as<float>();
    weight_inf_ = config["weight_inf"].as<float>();

    sortParticles();
}

PICFLIPFluid::~PICFLIPFluid() {
    delete grid_;
    delete vel_grid_;
    delete buffer_vel_grid_;
    delete orig_vel_grid_;
    delete divergence_grid_;
    delete pressure_grid_;
    delete buffer_pressure_grid_;
}

template <class T> T PICFLIPFluid::interpolate(const VecGrid<T>* grid, const glm::vec3& pos) {
    T result = T(0.0f);
    glm::ivec3 cell = glm::ivec3(pos);
    glm::vec3 weight = pos - glm::vec3(cell);
    glm::vec3 weights[2] = {glm::vec3(1.0f) - weight, weight};
    glm::ivec3 offset;
    for (offset.x = 0; offset.x < 2; ++offset.x)
    for (offset.y = 0; offset.y < 2; ++offset.y)
    for (offset.z = 0; offset.z < 2; ++offset.z) {
        result += weights[offset.x].x * weights[offset.y].y * weights[offset.z].z * grid->cvec(cell + offset);
    }
    // std::cout << "cell: " << cell << " pos: " << pos << " result: " << result << std::endl;
    return result;
}

template <class T> glm::vec3 PICFLIPFluid::sampleVelocity(const VecGrid<T>* grid, const glm::vec3& pos) {
    glm::vec3 vel;
    vel.x = interpolate(grid, pos - glm::vec3(0.0f, 0.5f, 0.5f)).x;
    vel.y = interpolate(grid, pos - glm::vec3(0.5f, 0.0f, 0.5f)).y;
    vel.z = interpolate(grid, pos - glm::vec3(0.5f, 0.5f, 0.0f)).z;
    return vel;
}

float PICFLIPFluid::scheduler(float t) {
    if (t < 0.0f) return 0.0f;
    float ret;
    // ret = scheduler_scale_ / (1.0f + exp(- t / scheduler_temperature_));
    ret = scheduler_temperature_ * t;
    ret = (ret > scheduler_scale_) ? scheduler_scale_ : ret;
    return ret;
}

void PICFLIPFluid::sortParticles() {
    std::vector<int> *cell_particles = new std::vector<int>[grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z];
    for (int i = 0; i < particles_.size(); ++i) {
        glm::ivec3 grid_pos = grid_->worldToGridInt(particles_[i].position);
        int cell_id = grid_pos.x + grid_pos.y * grid_->config_.resolution.x + grid_pos.z * grid_->config_.resolution.x * grid_->config_.resolution.y;
        cell_particles[cell_id].push_back(i);
    }
    int cnt = 0;
    std::vector<Particle> sorted_particles;
    for (int i = 0; i < grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z; ++i) {
        grid_->cells_[i].start = cnt;
        for (int j = 0; j < cell_particles[i].size(); ++j) {
            sorted_particles.push_back(particles_[cell_particles[i][j]]);
            ++cnt;
        }
        grid_->cells_[i].end = cnt;
    }
    particles_ = sorted_particles;
    delete[] cell_particles;
}

void PICFLIPFluid::transferToGrid() {
    glm::ivec3 resolution = grid_->config_.resolution;
    glm::ivec3 offset, cell;
    for (cell.x = 0; cell.x < resolution.x; ++cell.x)
    for (cell.y = 0; cell.y < resolution.y; ++cell.y)
    for (cell.z = 0; cell.z < resolution.z; ++cell.z) {
        glm::vec4 weight(0.0f);
        for (offset.x = cell.x - 1 > 0 ? cell.x - 1 : 0; offset.x <= cell.x + 1 && offset.x < resolution.x; ++offset.x)
        for (offset.y = cell.y - 1 > 0 ? cell.y - 1 : 0; offset.y <= cell.y + 1 && offset.y < resolution.y; ++offset.y)
        for (offset.z = cell.z - 1 > 0 ? cell.z - 1 : 0; offset.z <= cell.z + 1 && offset.z < resolution.z; ++offset.z) {
            int start = grid_->cell(offset).start;
            int end = grid_->cell(offset).end;
            for (int i = start; i < end; ++i) {
                glm::vec3 vel = particles_[i].velocity;
                glm::vec3 pos = grid_->worldToGrid(particles_[i].position) - glm::vec3(cell);
                glm::vec4 cur_weight = glm::vec4(
                    k(pos - glm::vec3(0.0f, 0.5f, 0.5f)), 
                    k(pos - glm::vec3(0.5f, 0.0f, 0.5f)), 
                    k(pos - glm::vec3(0.5f, 0.5f, 0.0f)), 
                    k(pos - glm::vec3(0.5f, 0.5f, 0.5f))
                );
                // std::cout<<cur_weight<<std::endl;
                vel_grid_->vec(cell).x += cur_weight.x * vel.x;
                vel_grid_->vec(cell).y += cur_weight.y * vel.y;
                vel_grid_->vec(cell).z += cur_weight.z * vel.z;
                weight += cur_weight;
            }
        }
        vel_grid_->vec(cell).x = (weight.x > weight_inf_) ? vel_grid_->vec(cell).x / weight.x : 0.0f;
        vel_grid_->vec(cell).y = (weight.y > weight_inf_) ? vel_grid_->vec(cell).y / weight.y : 0.0f;
        vel_grid_->vec(cell).z = (weight.z > weight_inf_) ? vel_grid_->vec(cell).z / weight.z : 0.0f;

        vel_grid_->vec(cell).w = weight.w;

        orig_vel_grid_->vec(cell) = vel_grid_->vec(cell);
    }
}

void PICFLIPFluid::marker() {
    glm::ivec3 resolution = grid_->config_.resolution;
    glm::ivec3 cell;
    for (cell.x = 0; cell.x < resolution.x; ++cell.x)
    for (cell.y = 0; cell.y < resolution.y; ++cell.y)
    for (cell.z = 0; cell.z < resolution.z; ++cell.z) {
        grid_->cell(cell).marker = bool(grid_->cell(cell).end - grid_->cell(cell).start > 0);
    }
}

void PICFLIPFluid::addForce(const glm::vec3& a, float dt) {
    glm::ivec3 resolution = grid_->config_.resolution;
    glm::ivec3 cell;
    for (cell.x = 0; cell.x < resolution.x; ++cell.x)
    for (cell.y = 0; cell.y < resolution.y; ++cell.y)
    for (cell.z = 0; cell.z < resolution.z; ++cell.z) { 
        vel_grid_->vec(cell).x += a.x * dt;
        vel_grid_->vec(cell).y += a.y * dt;
        vel_grid_->vec(cell).z += a.z * dt;
        if (cell.x == 0 || cell.x == resolution.x - 1) vel_grid_->vec(cell).x = 0.0f;
        if (cell.z == 0 || cell.z == resolution.z - 1) vel_grid_->vec(cell).z = 0.0f;
        if (cell.y == 0) vel_grid_->vec(cell).y = 0.0f;
        if (cell.y == resolution.y - 1 && vel_grid_->vec(cell).y > 0.0f) vel_grid_->vec(cell).y = 0.0f;
    }
}

void PICFLIPFluid::divergence() {
    glm::ivec3 cell, resolution = grid_->config_.resolution;
    for (cell.x = 0; cell.x < resolution.x; ++cell.x)
    for (cell.y = 0; cell.y < resolution.y; ++cell.y)
    for (cell.z = 0; cell.z < resolution.z; ++cell.z) {
        if (!grid_->cell(cell).marker) continue;
        glm::vec4 vel_min = vel_grid_->vec(cell);
        glm::vec3 vel_max;
        vel_max.x = vel_grid_->cvec(cell + glm::ivec3(1, 0, 0)).x;
        vel_max.y = vel_grid_->cvec(cell + glm::ivec3(0, 1, 0)).y;
        vel_max.z = vel_grid_->cvec(cell + glm::ivec3(0, 0, 1)).z;

        divergence_grid_->vec(cell) = glm::dot(vel_max - glm::vec3(vel_min), glm::vec3(1.0f));
        divergence_grid_->vec(cell) -= scheduler(vel_min.w / particles_per_cell_ - 1.0f);
    }
}

void PICFLIPFluid::jacobi(int iter) {
    glm::ivec3 cell, resolution = grid_->config_.resolution;
    for (int i = 0; i < iter; ++i) {
        for (cell.x = 0; cell.x < resolution.x; ++cell.x)
        for (cell.y = 0; cell.y < resolution.y; ++cell.y)
        for (cell.z = 0; cell.z < resolution.z; ++cell.z) {
            if (!grid_->cell(cell).marker) continue;
            buffer_pressure_grid_->vec(cell) = - divergence_grid_->vec(cell);
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell + glm::ivec3(1, 0, 0));
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell - glm::ivec3(1, 0, 0));
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell + glm::ivec3(0, 1, 0));
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell - glm::ivec3(0, 1, 0));
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell + glm::ivec3(0, 0, 1));
            buffer_pressure_grid_->vec(cell) += pressure_grid_->cvec(cell - glm::ivec3(0, 0, 1));
            buffer_pressure_grid_->vec(cell) /= 6.0f;
        }
        swapPressureBuffers();
    }
}

void PICFLIPFluid::subtractVel(void) {
    glm::ivec3 cell, resolution = grid_->config_.resolution;
    for (cell.x = 0; cell.x < resolution.x; ++cell.x)
    for (cell.y = 0; cell.y < resolution.y; ++cell.y)
    for (cell.z = 0; cell.z < resolution.z; ++cell.z) {
        float pres_max = pressure_grid_->vec(cell);
        glm::vec4 gradient;
        gradient.x = pres_max - pressure_grid_->cvec(cell - glm::ivec3(1, 0, 0));
        gradient.y = pres_max - pressure_grid_->cvec(cell - glm::ivec3(0, 1, 0));
        gradient.z = pres_max - pressure_grid_->cvec(cell - glm::ivec3(0, 0, 1));
        gradient.w = 0.0f;
        vel_grid_->vec(cell) -= gradient;
    }
}

void PICFLIPFluid::transferToParticles(float dt) {
    for (int index = 0; index < particles_.size(); ++index) {
        glm::vec3 cell_pos = grid_->worldToGrid(particles_[index].position);

        glm::vec3 particle_vel = particles_[index].velocity;
        glm::vec3 grid_vel = sampleVelocity(vel_grid_, cell_pos);
        glm::vec3 grid_vel_orig = sampleVelocity(orig_vel_grid_, cell_pos);

        glm::vec3 grid_change = grid_vel - grid_vel_orig;

        glm::vec3 flip_vel = particle_vel + grid_change;
        glm::vec3 pic_vel = grid_vel;

        glm::vec3 new_vel = pic_vel * (1.0f - flipness_) + flip_vel * flipness_;
        new_vel *= vel_discount_;

        glm::vec3 cfl = glm::vec3(1.0f, 1.0f, 1.0f) / (grid_->config_.scale * dt);
        new_vel.x = new_vel.x > cfl.x ? cfl.x : (new_vel.x < -cfl.x ? -cfl.x : new_vel.x);
        new_vel.y = new_vel.y > cfl.y ? cfl.y : (new_vel.y < -cfl.y ? -cfl.y : new_vel.y);
        new_vel.z = new_vel.z > cfl.z ? cfl.z : (new_vel.z < -cfl.z ? -cfl.z : new_vel.z);

        particles_[index].velocity = new_vel;
    }
}

void PICFLIPFluid::advect(float dt) {
    for (int index = 0; index < particles_.size(); ++index) {
        glm::vec3 cell_pos = grid_->worldToGrid(particles_[index].position);

        glm::vec3 grid_vel = sampleVelocity(vel_grid_, cell_pos);

        glm::vec3 halfway_pos = particles_[index].position + grid_vel * dt * 0.5f;
        cell_pos = grid_->worldToGrid(halfway_pos);
        glm::vec3 halfway_vel = sampleVelocity(vel_grid_, cell_pos);

        glm::vec3 step = (halfway_vel * dt);

        glm::vec3 new_pos = particles_[index].position + step;

        cell_pos = grid_->worldToGrid(new_pos);
        
        float wall_thickness = grid_->config_.wall_thickness;
        cell_pos.x = cell_pos.x < wall_thickness ? wall_thickness : (cell_pos.x > grid_->config_.resolution.x - wall_thickness ? grid_->config_.resolution.x - wall_thickness : cell_pos.x);
        cell_pos.y = cell_pos.y < wall_thickness ? wall_thickness : (cell_pos.y > grid_->config_.resolution.y - wall_thickness ? grid_->config_.resolution.y - wall_thickness : cell_pos.y);
        cell_pos.z = cell_pos.z < wall_thickness ? wall_thickness : (cell_pos.z > grid_->config_.resolution.z - wall_thickness ? grid_->config_.resolution.z - wall_thickness : cell_pos.z);

        // if (cell_pos.x < wall_thickness) cell_pos.x = wall_thickness, particles_[index].velocity.x = 0.0f;
        // if (cell_pos.x > grid_->config_.resolution.x - wall_thickness) cell_pos.x = grid_->config_.resolution.x - wall_thickness, particles_[index].velocity.x = 0.0f;
        // if (cell_pos.y < wall_thickness) cell_pos.y = wall_thickness, particles_[index].velocity.y = 0.0f;
        // if (cell_pos.y > grid_->config_.resolution.y - wall_thickness) cell_pos.y = grid_->config_.resolution.y - wall_thickness, particles_[index].velocity.y = 0.0f;
        // if (cell_pos.z < wall_thickness) cell_pos.z = wall_thickness, particles_[index].velocity.z = 0.0f;
        // if (cell_pos.z > grid_->config_.resolution.z - wall_thickness) cell_pos.z = grid_->config_.resolution.z - wall_thickness, particles_[index].velocity.z = 0.0f;

        particles_[index].position = grid_->gridToWorld(cell_pos);
    }
}
void PICFLIPFluid::update(float dt) {
    memset(grid_->cells_, 0, grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z * sizeof(Cell));
    memset(vel_grid_->vecs_, 0, grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z * sizeof(glm::vec3));
    memset(divergence_grid_->vecs_, 0, grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z * sizeof(float));
    memset(pressure_grid_->vecs_, 0, grid_->config_.resolution.x * grid_->config_.resolution.y * grid_->config_.resolution.z * sizeof(float));

    sortParticles();
    transferToGrid();
    marker();
    addForce(accerleration_, dt);
    divergence();
    jacobi(jacobi_iters_);
    subtractVel();
    transferToParticles(dt);
    advect(dt);
}