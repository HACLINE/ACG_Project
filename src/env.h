#ifndef ENV_H
#define ENV_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include "geometry.h"
#include "rigid_body.h"
#include "fluid.h"

class Simulation {
    public:
        Simulation();
        Simulation(YAML::Node, YAML::Node);
        ~Simulation();

        void update(float dt);

        void addRigidbody(Rigidbody*);
        void addFluid(Fluid*);

        inline int getNumRigidbodies() const { return rigidbodies_.size(); }
        inline int getNumFluids() const { return fluids_.size(); }
        Rigidbody* getRigidbody(int i) const;
        Fluid* getFluid(int i) const;
        inline const std::vector<Rigidbody*>& getRigidbodies() const { return rigidbodies_; }
        inline const std::vector<Fluid*>& getFluids() const { return fluids_; }
    
    private:
        std::vector<Rigidbody*> rigidbodies_;
        std::vector<Fluid*> fluids_;

        glm::vec3 gravity_;
        glm::vec3 box_min_, box_max_;

        float restitution_, friction_;
};

#endif