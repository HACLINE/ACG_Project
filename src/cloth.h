#ifndef CLOTH_H
#define CLOTH_H

#include "geometry.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include "spatial_hash.h"
#include "xpbd_constraints.h"

class Cloth {
public:
    Cloth(const std::string& path, const YAML::Node& config, float kernel_radius, int hash_table_size);
    ~Cloth();

    virtual void update(float);
    void computeForces();
    void collisionWithTriangle(Triangle*, float);
    void collisionWithSphere(Sphere*, float);
    void selfCollision();
    void selfCorrectSpring();

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a);
    void applyGravity(const glm::vec3& g) {applyAcceleration(g);}
    void applyDamping();
    void applyForce(const glm::vec3& f, int i);
    void applyForce(const glm::vec3& f) { for (int i = 0; i < num_particles_; ++i) applyForce(f, i); }
    void setFix(int ind, bool fixed);

    inline const std::vector<Particle>& getParticles() const { return particles_; }
    inline const int getNumParticles() const { return num_particles_; }
    inline const int getNumFaces() const { return num_faces_; }
    inline const int getNumSprings() const { return num_springs_; }
    
    Mesh getMesh();

protected:
    int num_x_, num_z_;
    int num_particles_, num_springs_, num_faces_;
    std::vector<Particle> particles_;
    std::vector<bool> fixed_;
    std::vector<Spring> springs_;
    std::vector<Face> faces_;
    std::vector<glm::vec3> prev_acceleration_;
    std::vector<glm::vec3> force_buffer_;

    float damping_;
    SpatialHash hash_table_;
};

class XPBDCloth : public Cloth {
public:
    XPBDCloth(const std::string& path, const YAML::Node& config, float kernel_radius, int hash_table_size);
    ~XPBDCloth();

    void update(float dt) override;

    void addDistanceConstraint(int p1, int p2, float stiffness);
    void addBendingConstraint(int p1, int p2, int p3, float stiffness);

    void solveDistanceConstraint(float dt);
    void solveBendingConstraint(float dt);

private:
    int num_distance_constraints_, num_bending_constraints_;
    int iters_;

    std::vector<XPBD_DistanceConstraint> distance_constraints_;
    std::vector<XPBD_BendingConstraint> bending_constraints_;

    std::vector<glm::vec3> p_, delta_p_;
    std::vector<float> w_;
};

#endif