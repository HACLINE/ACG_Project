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
        Simulation(YAML::Node config);
        ~Simulation();

        void update(float dt);

        void addRigidbody(const Rigidbody&);
        void addFluid(const Fluid&);

        inline int getNumRigidbodies() const { return rigidbodies_.size(); }
        inline int getNumFluids() const { return fluids_.size(); }
        const Rigidbody& getRigidbody(int i) const;
        const Fluid& getFluid(int i) const;
        inline const std::vector<Rigidbody>& getRigidbodies() const { return rigidbodies_; }
        inline const std::vector<Fluid>& getFluids() const { return fluids_; }
    
    private:
        std::vector<Rigidbody> rigidbodies_;
        std::vector<Fluid> fluids_;
};

#endif