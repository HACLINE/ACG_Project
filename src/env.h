#ifndef ENV_H
#define ENV_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include "geometry.h"
#include "rigid_body.h"
#include "fluid.h"
#include "cloth.h"

class Simulation {
    public:
        Simulation();
        Simulation(YAML::Node, YAML::Node, YAML::Node);
        ~Simulation();

        void update(float dt);

        void addRigidbody(Rigidbody*);
        void addFluid(Fluid*);
        void addCloth(Cloth*);
        void addWall(Triangle*);
        // void addWall(Sphere);

        inline int getNumRigidbodies(void) const { return rigidbodies_.size(); }
        inline int getNumFluids(void) const { return fluids_.size(); }
        inline int getNumCloths(void) const { return cloths_.size(); }
        inline int getNumWalls(void) const { return walls_.size(); }
        Rigidbody* getRigidbody(int i) const;
        Fluid* getFluid(int i) const;
        Cloth* getCloth(int i) const;
        Triangle* getWall(int i) const;
        inline const std::vector<Rigidbody*>& getRigidbodies() const { return rigidbodies_; }
        inline const std::vector<Fluid*>& getFluids() const { return fluids_; }
        inline const std::vector<Cloth*>& getCloths() const { return cloths_; }
        inline const std::vector<Triangle*>& getWalls() const { return walls_; }

        RenderObject getRenderObject(void);
    
    private:
        std::vector<Rigidbody*> rigidbodies_;
        std::vector<Fluid*> fluids_;
        std::vector<Cloth*> cloths_;
        std::vector<Triangle*> walls_;

        glm::vec3 gravity_;
        glm::vec3 box_min_, box_max_;

        float restitution_, friction_;

        YAML::Node cuda_;
};

#endif