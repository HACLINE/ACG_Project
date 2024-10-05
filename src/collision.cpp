#include "collision.h"
#include "geometry.h"
#include <iostream>

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