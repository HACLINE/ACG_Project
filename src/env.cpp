#include "env.h"
#include <string>
#include <iostream>
#include "collision.h"

Simulation::Simulation() {}

Simulation::Simulation(YAML::Node load, YAML::Node physics, YAML::Node cuda): cuda_(cuda) {
    std::string rigidbody_path = load["cwd"].as<std::string>() + load["rigidbody"]["path"].as<std::string>() + "/";
    std::string fluid_path = load["cwd"].as<std::string>() + load["fluid"]["path"].as<std::string>() + "/";
    std::string cloth_path = load["cwd"].as<std::string>() + load["cloth"]["path"].as<std::string>() + "/";
    for (int i = 0; i < load["rigidbody"]["cfg"].size(); ++i) {
        Rigidbody* rigidbody;
        if (load["rigidbody"]["type"].as<std::string>() == "impulse") {
            rigidbody = new ImpulseBasedRigidbody(rigidbody_path, load["rigidbody"]["cfg"][i], load["rigidbody"]["type"].as<std::string>());
        } else {
            std::cerr << "[Error] Invalid rigidtype" << std::endl;
            exit(1);
        }
        addRigidbody(rigidbody);
    }
    for (int i = 0; i < load["fluid"]["cfg"].size(); ++i) {
        Fluid* fluid;
        if (load["fluid"]["type"].as<std::string>() == "DFSPH") {
            fluid = new DFSPHFluid(fluid_path, load["fluid"]["cfg"][i], load["fluid"]["type"].as<std::string>(), cuda);
        } else {
            std::cerr << "[Error] Invalid fluidtype" << std::endl;
            exit(1);
        }
        addFluid(fluid);
    }
    for (int i = 0; i < load["cloth"]["cfg"].size(); ++i) {
        Cloth* cloth;
        if (load["cloth"]["type"].as<std::string>() == "mass_spring") {
            cloth = new Cloth(cloth_path, load["cloth"]["cfg"][i], load["cloth"]["kernel_radius"].as<float>(), load["cloth"]["hash_table_size"].as<int>());
        } else {
            std::cerr << "[Error] Invalid clothtype" << std::endl;
            exit(1);
        }
        addCloth(cloth);
    }
    for (int i = 0; i < load["wall"]["cfg"].size(); ++i) {
        if (load["wall"]["cfg"][i]["type"].as<std::string>() == "triangle") {
            Triangle* triangle = new Triangle{
                glm::vec3(load["wall"]["cfg"][i]["pos1"][0].as<float>(), load["wall"]["cfg"][i]["pos1"][1].as<float>(), load["wall"]["cfg"][i]["pos1"][2].as<float>()),
                glm::vec3(load["wall"]["cfg"][i]["pos2"][0].as<float>(), load["wall"]["cfg"][i]["pos2"][1].as<float>(), load["wall"]["cfg"][i]["pos2"][2].as<float>()),
                glm::vec3(load["wall"]["cfg"][i]["pos3"][0].as<float>(), load["wall"]["cfg"][i]["pos3"][1].as<float>(), load["wall"]["cfg"][i]["pos3"][2].as<float>()),
                load["wall"]["cfg"][i]["thickness"].as<float>(),
                load["wall"]["cfg"][i]["restitution"].as<float>(),
                load["wall"]["cfg"][i]["friction"].as<float>()
            };
            addWall(triangle);
        } else {
            std::cerr << "[Error] Invalid walltype" << std::endl;
            exit(1);
        }
    }

    gravity_ = glm::vec3(physics["gravity"][0].as<float>(), physics["gravity"][1].as<float>(), physics["gravity"][2].as<float>());
    box_min_ = glm::vec3(physics["box"]["min"][0].as<float>(), physics["box"]["min"][1].as<float>(), physics["box"]["min"][2].as<float>());
    box_max_ = glm::vec3(physics["box"]["max"][0].as<float>(), physics["box"]["max"][1].as<float>(), physics["box"]["max"][2].as<float>());
    restitution_ = physics["restitution"].as<float>();
    friction_ = physics["friction"].as<float>();
}

Simulation::~Simulation() {
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        delete rigidbodies_[i];
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        delete fluids_[i];
    }
    for (int i = 0; i < cloths_.size(); ++i) {
        delete cloths_[i];
    }
    for (int i = 0; i < walls_.size(); ++i) {
        delete walls_[i];
    }
}

void Simulation::addRigidbody(Rigidbody* rigidbody) {
    rigidbodies_.push_back(rigidbody);
}

void Simulation::addFluid(Fluid* fluid) {
    fluids_.push_back(fluid);
}

void Simulation::addCloth(Cloth* cloth) {
    cloths_.push_back(cloth);
}

void Simulation::addWall(Triangle* tri) {
    walls_.emplace_back(tri);
}

void Simulation::update(float dt) {
    // char x;
    // std::cin >> x;
    // Apply gravity
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i]->applyGravity(gravity_);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        fluids_[i]->applyGravity(gravity_);
    }
    for (int i = 0; i < cloths_.size(); ++i) {
        cloths_[i]->applyGravity(gravity_);
    }

    for (int i = 0; i < rigidbodies_.size(); ++i) {
        rigidbodies_[i]->update(dt);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        fluids_[i]->update(dt);
    }
    for (int i = 0; i < cloths_.size(); ++i) {
        cloths_[i]->update(dt);
    }
    
    // Collision detection
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        collision::rigidbody_box_collision(rigidbodies_[i], box_min_, box_max_, restitution_, friction_, cuda_);
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        collision::fluid_box_collision(fluids_[i], box_min_, box_max_, restitution_, friction_, cuda_);
    }
    for (int i = 0; i < cloths_.size(); ++i) { // Cloth-Wall collision
        cloths_[i]->selfCollision();
        for (int j = 0; j < walls_.size(); ++j) {
            cloths_[i]->collisionWithTriangle(walls_[j], dt);
        }
    }
}

Rigidbody* Simulation::getRigidbody(int i) const {
    assert(i < rigidbodies_.size());
    return rigidbodies_[i];
}

Fluid* Simulation::getFluid(int i) const {
    assert(i < fluids_.size());
    return fluids_[i];
}

Cloth* Simulation::getCloth(int i) const {
    assert(i < cloths_.size());
    return cloths_[i];
}

Triangle* Simulation::getWall(int i) const {
    assert(i < walls_.size());
    return walls_[i];
}

RenderObject Simulation::getRenderObject(void) {
    RenderObject render_object;
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        render_object.meshes.push_back(rigidbodies_[i]->getCurrentMesh());
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        render_object.particles.push_back(fluids_[i]->getParticles());
    }
    return render_object;
}