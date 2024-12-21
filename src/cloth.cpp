#include "cloth.h"
#include <fstream>
#include <iostream>
#include <chrono>

#include "utils.h"

Cloth::~Cloth() {
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaFree(cuda_particles_);
        cudaFree(cuda_face_norms_);
        cudaFree(cuda_vertex_norms_);
        cudaFree(cuda_faces_);
        cudaFree(cuda_fixed_);
    }
#endif
}

Cloth::Cloth(const std::string& path, const YAML::Node& config, const YAML::Node& cuda, float kernel_radius, int hash_table_size): particles_(), hash_table_(kernel_radius, hash_table_size, particles_), cuda_enabled_(cuda["enabled"].as<bool>()), cuda_block_size_(cuda["block_size"].as<int>()) {
    if (config["init"].as<std::string>() == "fromfile") {
        throw std::runtime_error("Not Implemented: Cloth init from file");
        // std::ifstream fin;
        // std::string filename = path + config["name"].as<std::string>();
        // fin.open(filename);
        // if (!fin) {
        //     std::cerr << "Failed to open file: " << filename << std::endl;
        //     return;
        // }
        // int opt;
        // while (fin >> opt) {
        //     if (opt == 0) { // particle
        //         float x, y, z, r, m;
        //         fin >> x >> y >> z >> r >> m;
        //         addVertex(glm::vec3(x, y, z), r, m);
        //     }
        //     else if (opt == 1) { // spring
        //         float k_s, k_d, rest_length;
        //         int p1, p2, type;
        //         fin >> p1 >> p2 >> k_s >> k_d >> rest_length >> type;
        //         springs_.push_back(Spring{p1, p2, k_s, k_d, rest_length, type});
        //     }
        //     else if (opt == 2) { // face (should add after particle!)
        //         int p1, p2, p3;
        //         fin >> p1 >> p2 >> p3;
        //         addFace(Face{p1, p2, p3});
        //     }
        // }
        // fixed_ = std::vector<bool>(particles_.size(), false);
        // fin.close();
        // std::cout << "[Load] Cloth: Load " << particles_.size() << " particles, " << springs_.size() << " springs from " << filename << std::endl;
    } else if (config["init"].as<std::string>() == "square") {
        int x_num = config["num"][0].as<int>(), z_num = config["num"][1].as<int>();
        num_x_ = x_num, num_z_ = z_num;
        float x_gap = config["scale"][0].as<float>() / (x_num - 1),
              z_gap = config["scale"][1].as<float>() / (z_num - 1);
        float slope_gap = sqrt(x_gap * x_gap + z_gap * z_gap);
        float mass = config["mass"].as<float>(); mass_ = mass;
        float x_start = - config["scale"][0].as<float>() / 2.0,
              z_start = - config["scale"][1].as<float>() / 2.0;
        float y_pos = config["height"].as<float>();
        float radius = config["radius"].as<float>();
        float dy = config["delta_y"].as<float>();
        for (int i = 0; i < x_num; ++i) {
            for (int j = 0; j < z_num; ++j) {
                addVertex(glm::vec3(x_start + i * x_gap, y_pos + dy * (float(x_num) / 2.0 - i) / float(x_num), z_start + j * z_gap), radius, mass);
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
                addFace(Face{((i+1) * z_num) + j, (i * z_num) + j, ((i+1) * z_num) + (j+1)});
                addFace(Face{(i * z_num) + (j+1), ((i+1) * z_num) + (j+1), (i * z_num) + j});
            }
        }

        fixed_ = new bool[particles_.size()];
        for (int i = 0; i < particles_.size(); ++i) fixed_[i] = false;
        if (config["fix_corner"].as<std::string>() == "true") {
            fixed_[0] = fixed_[z_num - 1] = fixed_[particles_.size() - 1] = fixed_[particles_.size() - z_num] = true;
            particles_[0].mass = particles_[z_num - 1].mass = particles_[particles_.size() - 1].mass = particles_[particles_.size() - z_num].mass = 999999.0f;
        } else if (config["fix_corner"].as<std::string>() == "half") {
            fixed_[0] = fixed_[z_num - 1] = true;
            particles_[0].mass = particles_[z_num - 1].mass = 999999.0f;
        }

        std::cout << "[Load] Cloth: Load " << particles_.size() << " particles and " << springs_.size() << " springs" << std::endl;
    }

    num_particles_ = particles_.size();
    num_springs_ = springs_.size();
    num_faces_ = faces_.size();
    damping_ = config["damping"].as<float>();
    wetting_speed_ = config["wetting_speed"].as<float>();
    // std::cout << "KR: "<<kernel_radius<<", HTS: "<<hash_table_size<<", NP: "<<num_particles_<<std::endl;
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaMalloc(&cuda_particles_, num_particles_ * sizeof(Particle));
        cudaMalloc(&cuda_face_norms_, num_faces_ * sizeof(glm::vec3));
        cudaMalloc(&cuda_vertex_norms_, num_particles_ * sizeof(glm::vec3));
        cudaMalloc(&cuda_faces_, num_faces_ * sizeof(Face));
        cudaMalloc(&cuda_fixed_, num_particles_ * sizeof(bool));

        cudaMemcpy(cuda_particles_, particles_.data(), num_particles_ * sizeof(Particle), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_face_norms_, face_norms_.data(), num_faces_ * sizeof(glm::vec3), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_vertex_norms_, vertex_norms_.data(), num_particles_ * sizeof(glm::vec3), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_faces_, faces_.data(), num_faces_ * sizeof(Face), cudaMemcpyHostToDevice);

        face_norms_ = std::vector<glm::vec3>();
        vertex_norms_ = std::vector<glm::vec3>();
        
        cudaMemcpy(cuda_fixed_, fixed_, num_particles_ * sizeof(bool), cudaMemcpyHostToDevice);
    }
#endif
}

void Cloth::addVertex(const glm::vec3& pos, float r, float m) {
    particles_.push_back(Particle{pos, glm::vec3(0.0f), glm::vec3(0.0f), r, m});
    force_buffer_.push_back(glm::vec3(0.0f));
    prev_acceleration_.push_back(glm::vec3(0.0f));
    vertex_norms_.push_back(glm::vec3(0.0f));
}

void Cloth::addFace(const Face& face) {
    faces_.push_back(face);
    face_norms_.push_back(glm::normalize(glm::cross(particles_[face.v2].position - particles_[face.v1].position, particles_[face.v3].position - particles_[face.v1].position)));
}

void Cloth::computeNormals() {
    for (int i = 0; i < num_faces_; ++i) {
        face_norms_[i] = glm::normalize(glm::cross(particles_[faces_[i].v2].position - particles_[faces_[i].v1].position, particles_[faces_[i].v3].position - particles_[faces_[i].v1].position));
    }
    for (int i = 0; i < num_particles_; ++i) {
        vertex_norms_[i] = glm::vec3(0.0f);
    }
    for (int i = 0; i < num_faces_; ++i) {
        vertex_norms_[faces_[i].v1] += face_norms_[i];
        vertex_norms_[faces_[i].v2] += face_norms_[i];
        vertex_norms_[faces_[i].v3] += face_norms_[i];
    }
    for (int i = 0; i < num_particles_; ++i) {
        vertex_norms_[i] = glm::normalize(vertex_norms_[i]);
    }
}

void Cloth::applyAcceleration(const glm::vec3& a) {
    for (int i = 0; i < num_particles_; ++i) {
        if (!fixed_[i]) {
            particles_[i].acceleration += a;
        }
    }
}

void Cloth::applyForce(const glm::vec3& f, int i) {
    if (!fixed_[i]) {
        force_buffer_[i] += f;
    }
}

void Cloth::applyDamping() {
    for (int i = 0; i < num_particles_; ++i) {
        if (!fixed_[i]) {
            particles_[i].velocity *= (1. - damping_);
        }
    }
}

void Cloth::collisionWithTriangle(Triangle* tri, float dt) {
    for (int i = 0; i < num_particles_; ++i) {
        glm::vec3 p = particles_[i].position, v = particles_[i].velocity;
        glm::vec3 n = glm::cross(tri->v2 - tri->v1, tri->v3 - tri->v1);
        n = glm::normalize(n);
        float dist = glm::dot(n, p - tri->v1), speed = - glm::dot(n, v);
        if (dist < 0.0f) {
            n = -n, dist = -dist, speed = -speed;
        }
        if (dist < tri->thickness) {
            // correct position
            particles_[i].position += (tri->thickness - dist) * n;
        }
        // if (speed > 0.0f) {
        //     float t = dist / speed;
        //     if (t < dt) { // have collision
        //         glm::vec3 collision_point = p + particles_[i].velocity * t;
        //         glm::vec3 normal_velocity = glm::dot(particles_[i].velocity, n) * n;
        //         glm::vec3 tangential_velocity = particles_[i].velocity - normal_velocity;

        //         glm::vec3 new_normal_velocity = - tri->restitution * normal_velocity;
        //         glm::vec3 new_tangential_velocity = tangential_velocity * (1.0f - tri->friction);

        //         particles_[i].velocity = new_normal_velocity + new_tangential_velocity;
        //         // particles_[i].position = collision_point;
        //     }
        // }
    }
}

void Cloth::collisionWithSphere(Sphere* sphere, float dt) {
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        collisionWithSphereCUDA(sphere, dt);
        cudaMemcpy(particles_.data(), cuda_particles_, num_particles_ * sizeof(Particle), cudaMemcpyDeviceToHost);
        return;
    }
#endif
    for (int i = 0; i < num_particles_; ++i) {
        glm::vec3 p = particles_[i].position;
        glm::vec3 n = glm::normalize(p - sphere->center);
        float dist = glm::length(p - sphere->center);
        if (dist < sphere->radius) {
            // correct position
            particles_[i].position += (sphere->radius - dist) * n;
        }
    }
}

void Cloth::selfCollision() {
    hash_table_.update();
    int total_cnt = 0;
    for (int i = 0; i < num_particles_; ++i) {
        std::vector<int> neighbors;
        hash_table_.getNeighbors(particles_[i].position, neighbors);
        int cnt = 0;
        glm::vec3 correction = glm::vec3(0.0f);
        for (int j: neighbors) {
            if (i == j) continue;
            glm::vec3 r = particles_[i].position - particles_[j].position;
            float dist = glm::length(r);
            if (dist < particles_[i].radius + particles_[j].radius) {
                glm::vec3 n = glm::normalize(r);
                correction += (particles_[i].radius + particles_[j].radius - dist) * n;
                cnt++;
            }
        }
        if (cnt > 0) {
            particles_[i].position += correction / float(cnt) / 5.0f;
            total_cnt += cnt;
        }
    }
    // std::cout << "Total collision: " << total_cnt << std::endl;
}

void Cloth::selfCorrectSpring() {
    for (int i = 0; i < num_springs_; ++i) {
        float dif = glm::length(particles_[springs_[i].p1].position - particles_[springs_[i].p2].position) - 1.1 * springs_[i].rest_length;
        if (dif > 0) {
            glm::vec3 correction = dif * glm::normalize(particles_[springs_[i].p1].position - particles_[springs_[i].p2].position);
            if (fixed_[springs_[i].p1]) {
                particles_[springs_[i].p2].position += correction;
            } else if (fixed_[springs_[i].p2]) {
                particles_[springs_[i].p1].position -= correction;
            } else {
                particles_[springs_[i].p1].position -= correction / 2.0f;
                particles_[springs_[i].p2].position += correction / 2.0f;
            }
        }
    }
}

void Cloth::update(float dt) {
    computeForces();

    // Euler Integration
    for (int i = 0; i < num_particles_; ++i) {
        if (!fixed_[i]) {
            // std::cout << particles_[i].acceleration.y << " ";
            // if (abs(particles_[i].velocity.y) > 20) exit(0);
            particles_[i].acceleration += force_buffer_[i] / particles_[i].mass;
            particles_[i].velocity += particles_[i].acceleration * dt;
            particles_[i].position += particles_[i].velocity * dt;
            // prev_acceleration_[i] = particles_[i].acceleration;
        }
        if (fixed_[i]) {
            particles_[i].velocity = glm::vec3(0.0f);
            // prev_acceleration_[i] = glm::vec3(0.0f);
        }
        particles_[i].acceleration = force_buffer_[i] = glm::vec3(0.0f);
    }
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

Mesh Cloth::getMesh() {
    Mesh mesh;
    mesh.vertices.clear();
    mesh.faces = faces_;
    for (int i = 0; i < num_particles_; ++i) {
        mesh.vertices.push_back(Vertex{particles_[i].position});
    }
    return mesh;
}

std::vector<float> Cloth::getWettings() {
    std::vector<float> wettings;
    wettings.clear();
    for (int i = 0; i < num_particles_; ++i) {
        wettings.push_back(particles_[i].wetting);
    }
    return wettings;
}

void Cloth::wettingDiffusion(float dt) {
    int dx[4] = {0, 0, 1, -1}, dz[4] = {1, -1, 0, 0};
    float coeff = dt * wetting_speed_;
    std::vector<std::pair<float, float> > new_wettings(num_particles_, std::make_pair(0.0f, 0.0f));
    for (int i = 0; i < num_z_; ++i) {
        for (int j = 0; j < num_x_; ++j) {
            int ind = i * num_z_ + j;
            float sum = 0.0f;
            float cnt = 0.0f;
            for (int k = 0; k < 4; ++k) {
                int x = j + dx[k], z = i + dz[k];
                if (x >= 0 && x < num_x_ && z >= 0 && z < num_z_) {
                    sum += particles_[z * num_z_ + x].wetting;
                    cnt += 1.0f;
                }
            }
            new_wettings[ind] = std::make_pair(sum, cnt);
        }
    }
    for (int i = 0; i < num_particles_; ++i) {
        particles_[i].wetting = (1.0 - new_wettings[i].second * coeff) * particles_[i].wetting + new_wettings[i].first * coeff;
    }
}

XPBDCloth::XPBDCloth(const std::string& path, const YAML::Node& config, const YAML::Node& cuda, float kernel_radius, int hash_table_size): Cloth(path, config, cuda, kernel_radius, hash_table_size) {
    num_distance_constraints_ = num_bending_constraints_ = 0;
    for (int i = 0; i < num_x_; ++i) {
        for (int j = 0; j < num_z_; ++j) {
            if (i < num_x_ - 1) {
                addDistanceConstraint(i * num_z_ + j, (i+1) * num_z_ + j, config["stiff"]["horizontal"].as<float>());
            }
            if (j < num_z_ - 1) {
                addDistanceConstraint(i * num_z_ + j, i * num_z_ + (j+1), config["stiff"]["vertical"].as<float>());
            }
            if (i < num_x_ - 1 && j < num_z_ - 1) {
                addDistanceConstraint(i * num_z_ + j, (i+1) * num_z_ + (j+1), config["stiff"]["shear"].as<float>());
                addDistanceConstraint(i * num_z_ + (j+1), (i+1) * num_z_ + j, config["stiff"]["shear"].as<float>());
            }
            if (i < num_x_ - 2) {
                addBendingConstraint(i * num_z_ + j, (i+1) * num_z_ + j, (i+2) * num_z_ + j, config["stiff"]["bend"].as<float>());
            }
            if (j < num_z_ - 2) {
                addBendingConstraint(i * num_z_ + j, i * num_z_ + (j+1), i * num_z_ + (j+2), config["stiff"]["bend"].as<float>());
            }
        }
    }
    for (int i = 0; i < num_particles_; ++i) {
        w_.push_back(1.0f / particles_[i].mass);
    }
    iters_ = config["iters"].as<int>();
    p_ = delta_p_ = std::vector<glm::vec3>(num_particles_, glm::vec3(0.0f));

#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaMalloc(&cuda_distance_constraints_, num_distance_constraints_ * sizeof(XPBD_DistanceConstraint));
        cudaMalloc(&cuda_bending_constraints_, num_bending_constraints_ * sizeof(XPBD_BendingConstraint));
        cudaMalloc(&cuda_p_, num_particles_ * sizeof(glm::vec3));
        cudaMalloc(&cuda_delta_p_, num_particles_ * sizeof(glm::vec3));
        cudaMalloc(&cuda_w_, num_particles_ * sizeof(float));

        cudaMemcpy(cuda_distance_constraints_, distance_constraints_.data(), num_distance_constraints_ * sizeof(XPBD_DistanceConstraint), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_bending_constraints_, bending_constraints_.data(), num_bending_constraints_ * sizeof(XPBD_BendingConstraint), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_p_, p_.data(), num_particles_ * sizeof(glm::vec3), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_delta_p_, delta_p_.data(), num_particles_ * sizeof(glm::vec3), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_w_, w_.data(), num_particles_ * sizeof(float), cudaMemcpyHostToDevice);

        p_ = delta_p_ = std::vector<glm::vec3>();
        w_ = std::vector<float>();
    }
#endif
}

XPBDCloth::~XPBDCloth() {
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaFree(cuda_distance_constraints_);
        cudaFree(cuda_bending_constraints_);
    }
#endif
}

void XPBDCloth::addDistanceConstraint(int p1, int p2, float stiffness) {
    distance_constraints_.push_back(XPBD_DistanceConstraint{p1, p2, glm::length(particles_[p1].position - particles_[p2].position), stiffness});
    num_distance_constraints_++;
}

void XPBDCloth::addBendingConstraint(int p1, int p2, int p3, float stiffness) {
    glm::vec3 avg = (particles_[p1].position + particles_[p2].position + particles_[p3].position) / 3.0f;
    float rest_length = glm::length(avg - particles_[p3].position);
    bending_constraints_.push_back(XPBD_BendingConstraint{p1, p2, p3, rest_length, stiffness});
    num_bending_constraints_++;
}

void XPBDCloth::solveDistanceConstraint(float dt) {
    for (int i = 0; i < num_distance_constraints_; ++i) {
        int p1 = distance_constraints_[i].p1, p2 = distance_constraints_[i].p2;
        float rest_length = distance_constraints_[i].rest_length, k = distance_constraints_[i].stiffness;
        float alpha_tilde = k / dt / dt;
        float w = w_[p1] + w_[p2];
        glm::vec3 n = particles_[p1].position - particles_[p2].position;
        float dist = glm::length(n);
        n = glm::normalize(n);
        float C = dist - rest_length;
        float delta_lambda = - C / (w + alpha_tilde);
        delta_p_[p1] += delta_lambda * w_[p1] * n;
        delta_p_[p2] -= delta_lambda * w_[p2] * n;
    }
}

void XPBDCloth::solveBendingConstraint(float dt) {
    ;
}

void XPBDCloth::update(float dt) {
    // add external forces
#ifdef HAS_CUDA
    if (cuda_enabled_) {
        cudaMemcpy(cuda_particles_, particles_.data(), num_particles_ * sizeof(Particle), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_fixed_, fixed_, num_particles_ * sizeof(bool), cudaMemcpyHostToDevice);
        updateCUDA(dt);
        cudaMemcpy(particles_.data(), cuda_particles_, num_particles_ * sizeof(Particle), cudaMemcpyDeviceToHost);
        return;
    }
#endif
    for (int i = 0; i < num_particles_; ++i) {
        particles_[i].acceleration += force_buffer_[i] * w_[i];
    }
    // update position
    float ds = dt / iters_;
    for (int iter = 0; iter < iters_; ++iter) {
        for (int i = 0; i < num_particles_; ++i) {
            p_[i] = particles_[i].position;
            particles_[i].position += particles_[i].velocity * (1 - damping_) * ds + particles_[i].acceleration * ds * ds;
        }
        solveDistanceConstraint(ds);
        solveBendingConstraint(ds);
        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].position += delta_p_[i];
            delta_p_[i] = glm::vec3(0.0f);
            particles_[i].velocity = (particles_[i].position - p_[i]) / ds;
        }
    }

    for (int i = 0; i < num_particles_; ++i) {
        particles_[i].acceleration = glm::vec3(0.0f);
        force_buffer_[i] = glm::vec3(0.0f);
    }

    // wetting effect
    wettingDiffusion(dt);
}

void XPBDCloth::setFix(int ind, bool fixed) {
    assert(num_particles_ > ind && ind >= 0);
    // std::cout << "hi" << std::endl;
    fixed_[ind] = fixed;
    if (fixed) {
        particles_[ind].mass = 999999.0f;
        w_[ind] = 0.0f;
        particles_[ind].acceleration = particles_[ind].velocity = glm::vec3(0.0f);
        force_buffer_[ind] = glm::vec3(0.0f);
    } else {
        particles_[ind].mass = mass_;
        w_[ind] = 1.0f / mass_;
    }
    // std::cout << "bye" << std::endl;
}

void XPBDCloth::setFix(int x, int z, bool fixed) {
    assert(num_x_ > x && x >= 0 && num_z_ > z && z >= 0);
    setFix(x * num_z_ + z, fixed);
}