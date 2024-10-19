#ifndef CLOTH_H
#define CLOTH_H

#include "geometry.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include "spatial_hash.h"

class Cloth {
public:
    Cloth(const std::string& path, const YAML::Node& config, float kernel_radius, int hash_table_size);
    ~Cloth();

    void update(float);
    void computeForces();
    void collisionWithTriangle(Triangle*, float);
    void collisionWithSphere(Sphere*, float);
    void selfCollision();

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a) { for (int i = 0; i < num_particles_; ++i) particles_[i].acceleration += a; }
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

private:
    int num_particles_, num_springs_, num_faces_;
    std::vector<Particle> particles_;
    std::vector<bool> fixed_;
    std::vector<Spring> springs_;
    std::vector<Face> faces_;

    float damping_;
    SpatialHash hash_table_;
};

#endif