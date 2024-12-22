#include "simulation.h"
#include <string>
#include <iostream>
#include "collision.h"

Simulation::Simulation() {}

Simulation::Simulation(YAML::Node load, YAML::Node physics, YAML::Node cuda, YAML::Node blender): cuda_(cuda), blender_(blender), coupling_(physics["coupling"]) {
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
            fluid = new DFSPHFluid(fluid_path, load["fluid"]["cfg"][i], load["fluid"]["type"].as<std::string>(), physics, cuda);
        } else if (load["fluid"]["type"].as<std::string>() == "PICFLIP") {
            fluid = new PICFLIPFluid(fluid_path, load["fluid"]["cfg"][i], load["fluid"]["type"].as<std::string>(), physics, cuda);
        } else {
            std::cerr << "[Error] Invalid fluidtype" << std::endl;
            exit(1);
        }
        addFluid(fluid);
    }
    for (int i = 0; i < load["cloth"]["cfg"].size(); ++i) {
        Cloth* cloth;
        if (load["cloth"]["type"].as<std::string>() == "mass_spring") {
            cloth = new Cloth(cloth_path, load["cloth"]["cfg"][i], cuda, load["cloth"]["kernel_radius"].as<float>(), load["cloth"]["hash_table_size"].as<int>());
        } else if (load["cloth"]["type"].as<std::string>() == "XPBD") {
            cloth = new XPBDCloth(cloth_path, load["cloth"]["cfg"][i], cuda, load["cloth"]["kernel_radius"].as<float>(), load["cloth"]["hash_table_size"].as<int>());
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
        } else if (load["wall"]["cfg"][i]["type"].as<std::string>() == "sphere") {
            Sphere* sphere = new Sphere{
                glm::vec3(load["wall"]["cfg"][i]["center"][0].as<float>(), load["wall"]["cfg"][i]["center"][1].as<float>(), load["wall"]["cfg"][i]["center"][2].as<float>()),
                load["wall"]["cfg"][i]["radius"].as<float>()
            };
            addSphere(sphere);
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

#ifdef HAS_CUDA
    if (cuda["enabled"].as<bool>()) {
        int max_fluid = 0, max_cloth = 0;
        for (int i = 0; i < fluids_.size(); ++i) {
            max_fluid = max_fluid > fluids_[i]->getNumParticles() ? max_fluid : fluids_[i]->getNumParticles();
        }
        for (int i = 0; i < cloths_.size(); ++i) {
            max_cloth = max_cloth > cloths_[i]->getNumParticles() ? max_cloth : cloths_[i]->getNumParticles();
        }
        collision::coupling_init_CUDA(max_fluid, max_cloth, coupling_["neighborhood_size"].as<int>());
    }
#endif
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

void Simulation::addSphere(Sphere* sphere) {
    spheres_.emplace_back(sphere);
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

    for (int i = 0; i < fluids_.size(); ++i) {
        for (int j = 0; j < cloths_.size(); ++j) {
            collision::fluid_cloth_collision(fluids_[i], cloths_[j], dt, cuda_, coupling_);
        }
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
    // for (int i = 0; i < cloths_.size(); ++i) {
    //     for (int j = 0; j < walls_.size(); ++j) {
    //         cloths_[i]->collisionWithTriangle(walls_[j], dt);
    //     }
    //     for (int j = 0; j < spheres_.size(); ++j) {
    //         cloths_[i]->collisionWithSphere(spheres_[j], dt);
    //     }
    //     cloths_[i]->selfCollision();
    // }
}

// void Simulation::update(float dt, PanelInfo panel_info) { // Only support cloth for now!
//     for (int i = 0; i < cloths_.size(); ++i) {
//         if (panel_info.l_click_button) {
//             std::pair<int, int> cloth_grid = cloths_[i]->getNumGrids();
//             float posx = float(panel_info.click_x) / float(panel_info.window_x) * cloth_grid.first;
//             float posz = float(panel_info.click_y) / float(panel_info.window_y) * cloth_grid.second;
//             cloths_[i]->setFix(int(posx), int(posz), true);
//             // std::cout << "Fix " << int(posx) << " " << int(posz) << std::endl;
//         }
//         cloths_[i]->applyGravity(gravity_);
//     }

//     for (int i = 0; i < cloths_.size(); ++i) {
//         cloths_[i]->update(dt);
//     }
// }

void Simulation::update(float dt, PanelInfo* panel_info) { // Only support cloth for now!
    assert(panel_info != nullptr);
    for (int i = 0; i < cloths_.size(); ++i) {
        if (panel_info->l_click_button) {
            std::pair<int, int> cloth_grid = cloths_[i]->getNumGrids();
            float posx = float(panel_info->click_x) / float(panel_info->window_x) * cloth_grid.first;
            float posz = float(panel_info->click_y) / float(panel_info->window_y) * cloth_grid.second;
            cloths_[i]->setFix(int(posx), int(posz), true);
            // std::cout << "Fix " << int(posx) << " " << int(posz) << std::endl;
        }
        cloths_[i]->applyGravity(gravity_);
    }
    for (int i = 0; i < cloths_.size(); ++i) {
        cloths_[i]->update(dt);
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

Sphere* Simulation::getSphere(int i) const {
    assert(i < spheres_.size());
    return spheres_[i];
}

RenderObject Simulation::getRenderObject(void) {
    RenderObject render_object;
    for (int i = 0; i < rigidbodies_.size(); ++i) {
        render_object.meshes.push_back(rigidbodies_[i]->getCurrentMesh());
    }
    for (int i = 0; i < fluids_.size(); ++i) {
        render_object.particles.push_back(fluids_[i]->getParticles());
    }
    for (int i = 0; i < cloths_.size(); ++i) {
        render_object.meshes.push_back(cloths_[i]->getMesh());
    }
    return render_object;
}