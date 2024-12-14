#include "collision.h"
#include "geometry.h"
#include <iostream>
#include "spatial_hash.h"

namespace collision {
    Neighbourhood* fluid_cloth_neighbourhood = nullptr;
#ifdef HAS_CUDA
    VecWithInt *delta_cloth_positions = nullptr, *delta_old_cloth_positions = nullptr, *delta_fluid_positions = nullptr, *delta_fluid_velocities = nullptr;
#endif
}

//Impulse-based collision response
void collision::rigidbody_box_collision(Rigidbody* rigidbody, const glm::vec3& box_min, const glm::vec3& box_max, float restitution, float friction, const YAML::Node& cuda) {
    if (rigidbody->getType() == "impulse") {
        Mesh mesh = rigidbody->getCurrentMesh();
        glm::vec3 position = rigidbody->getPosition();
        glm::vec3 velocity = rigidbody->getVelocity();
        glm::vec3 angular_velocity = rigidbody->getAngularVelocity();
        restitution = rigidbody->restitution_schedule(restitution);
        // std::cout << "restitution: " << restitution << std::endl;

        glm::vec3 normal[6] = {
            glm::vec3(1.0f, 0.0f, 0.0f),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(0.0f, -1.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f)
        };
        float bias[6] = {
            -box_min.x,
            box_max.x,
            -box_min.y,
            box_max.y,
            -box_min.z,
            box_max.z
        };

        for (int i = 0; i < 6; ++i) {
            glm::vec3 r = glm::vec3(0.0f);
            int num = 0;
            for (auto vertex : mesh.vertices) {
                if (glm::dot(vertex.position, normal[i]) + bias[i] < 0.0f) {
                    r += vertex.position - position;
                    num++;
                }
            }
            if (num == 0) {
                continue;
            }
            r /= (float)num;
            // std::cout << "r: " << r.x << " " << r.y << " " << r.z << std::endl;

            glm::vec3 v = velocity + glm::cross(angular_velocity, r);
            if (glm::dot(v, normal[i]) >= 0.0f) {
                continue;
            }

            glm::vec3 vn = glm::dot(v, normal[i]) * normal[i];
            glm::vec3 vt = v - vn;
            glm::vec3 v_new = -restitution * vn + (1.0f - friction * (1 + restitution) * glm::length(vn) / (glm::length(vt) + 1e-6f) > 0.0f ? friction * (1 + restitution) * glm::length(vn) / glm::length(vt) : 0.0f) * vt;

            glm::mat3 K = 1.0f / rigidbody->getMass() * glm::mat3(1.0f) - cross_matrix(r) * rigidbody->getCurrentInvInertia() * cross_matrix(r);
            glm::vec3 j = glm::inverse(K) * (v_new - v);
            // std::cout << "v: " << v.x << " " << v.y << " " << v.z << std::endl;
            // std::cout << "vn: " << vn.x << " " << vn.y << " " << vn.z << std::endl;
            // std::cout << "vt: " << vt.x << " " << vt.y << " " << vt.z << std::endl;
            // std::cout << "v_new: " << v_new.x << " " << v_new.y << " " << v_new.z << std::endl;
            // std::cout << "j: " << j.x << " " << j.y << " " << j.z << std::endl;

            rigidbody->addtoVelocityBuffer(j / rigidbody->getMass());
            rigidbody->addtoAngularVelocityBuffer(rigidbody->getCurrentInvInertia() * glm::cross(r, j));
            rigidbody->doCollision();
        }
    } else {
        std::cerr << "[Error] Unsupported rigidbody type" << std::endl;
    }
    
}

void collision::fluid_box_collision(Fluid* fluid, const glm::vec3& box_min, const glm::vec3& box_max, float restitution, float friction, const YAML::Node& cuda) {
#ifdef HAS_CUDA
    if (cuda["enabled"].as<bool>()) {
        fluid_box_collision_CUDA(fluid, box_min, box_max, restitution, friction, cuda);
        return;
    }
#endif
     glm::vec3 normal[6] = {
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(-1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, -1.0f)
    };
    float bias[6] = {
        -box_min.x,
        box_max.x,
        -box_min.y,
        box_max.y,
        -box_min.z,
        box_max.z
    };
    std::vector<Particle>& particles = fluid->getParticles();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < particles.size(); ++j) {
            if (glm::dot(particles[j].position, normal[i]) + bias[i] < 0.0f) {
                particles[j].position += - (glm::dot(particles[j].position, normal[i]) + bias[i]) * normal[i];
                if (glm::dot(particles[j].velocity, normal[i]) < 0.0f) {
                    glm::vec3 vn = glm::dot(particles[j].velocity, normal[i]) * normal[i], vt = particles[j].velocity - vn;
                    float a = 1.0f - friction * (1 + restitution) * glm::length(vn) / (glm::length(vt) + 1e-6f);
                    fluid->addtoVelocityBuffer(-restitution * vn + (a > 0.0f ? friction * (1 + restitution) * glm::length(vn) / glm::length(vt) : 0.0f) * vt - particles[j].velocity, j);
                }
            }
        }
    }
}

void collision::fluid_cloth_collision(Fluid* fluid, Cloth* cloth, float dt, YAML::Node& cuda, YAML::Node& coupling) {
    std::string fluid_type = fluid->getType();
    std::string cloth_type = cloth->getType();

    if (fluid_type == "PICFLIP" && cloth_type == "XPBD") {
    #ifdef HAS_CUDA
        if (cuda["enabled"].as<bool>()) {
            PICFLIP_XPBD_collision_CUDA(static_cast<PICFLIPFluid*>(fluid), static_cast<XPBDCloth*>(cloth), dt, cuda, coupling);
            return;
        }
    #endif
        cloth->computeNormals();
        std::vector<Particle>& particles = fluid->getParticles();
        std::vector<Particle>& cloth_particles = cloth->getParticles();
        std::vector<glm::vec3>& cloth_old_positions = cloth->getOldPositions();
        std::vector<glm::vec3>& cloth_normals = cloth->getVertexNormals();
        assert(cloth_old_positions.size() == cloth_particles.size() && particles.size() > 0 && cloth_particles.size() > 0);
        float r_c = cloth_particles[0].radius;
        float r_p = particles[0].radius;
        float r = r_c + r_p;

        // std::cout << "r " << r << std::endl;

        auto collide_particles = [&](int fluid_idx, int cloth_idx) {
            glm::vec3 cp = cloth_particles[cloth_idx].position;
            glm::vec3 fp = particles[fluid_idx].position;
            glm::vec3 dir = cp - fp;
            float dist = glm::length(dir);
            if (dist < r) {
                float wetting = calc_wetting(dist, r);
                cloth_particles[cloth_idx].wetting += wetting;

                glm::vec3 normal = cloth_normals[cloth_idx];
                if (glm::dot(dir, normal) < 0.0f) {
                    normal = -normal;
                }
                glm::vec3 v0 = (cp - cloth_old_positions[cloth_idx]) / dt;
                glm::vec3 v1 = particles[fluid_idx].velocity;
                float w0 = 1.0f / cloth_particles[cloth_idx].mass;
                float w1 = 1.0f / particles[fluid_idx].mass;
                float iconstraintMass = 1.0f / (w0 + w1);
                glm::vec3 dv = v0 - v1;
                if (glm::dot(dv, normal) < 0.0f) {
                    float jn = - glm::dot(dv, normal) * iconstraintMass;
                    v0 += normal * (jn * w0);
                    v1 -= normal * (jn * w1);
                    particles[fluid_idx].velocity = v1;
                }
                float pen = r - glm::dot(cp, normal) + glm::dot(fp, normal);
                pen *= iconstraintMass;
                cp += normal * pen * w0;
                cloth_particles[cloth_idx].position = cp;
                cloth_old_positions[cloth_idx] = cp - v0 * dt;
                particles[fluid_idx].position = fp - normal * pen * w1;
            }
        };

        SpatialHash hash_table(r, coupling["neighborhood_size"].as<int>(), cloth_particles);
        hash_table.update();
        for (int i = 0; i < particles.size(); ++i) {
            std::vector<int> neighbors;
            hash_table.getNeighbors(particles[i].position, neighbors);
            for (int j : neighbors) {
                collide_particles(i, j);
            }
        }

    } else {
        std::cout << "[Simulation ERROR] Unsupported fluid-cloth collision of fluid type: " << fluid_type << ", cloth type: " << cloth_type << std::endl;
        exit(1);
    }
}
