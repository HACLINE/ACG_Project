#ifndef CLOTH_H
#define CLOTH_H

#include "geometry.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include "spatial_hash.h"
#include "xpbd_constraints.h"

class Cloth {
public:
    Cloth(const std::string& path, const YAML::Node& config, const YAML::Node& cuda, float kernel_radius, int hash_table_size);
    ~Cloth();
    virtual std::string getType() const { return "mass_spring"; }

    virtual void update(float);
    void computeForces();
    void collisionWithTriangle(Triangle*, float);
    void collisionWithSphere(Sphere*, float);
    void selfCollision();
    void selfCorrectSpring();
#ifdef HAS_CUDA
    void collisionWithSphereCUDA(Sphere*, float);
#endif

    void applyAcceleration(const glm::vec3& a, int i) { particles_[i].acceleration += a; }
    void applyAcceleration(const glm::vec3& a);
#ifdef HAS_CUDA
    void applyGravity(const glm::vec3& g) { if (cuda_enabled_) applyAccelerationCUDA(g); else applyAcceleration(g); }
#else
    void applyGravity(const glm::vec3& g) {applyAcceleration(g);}
#endif
    void applyDamping();
    void applyForce(const glm::vec3& f, int i);
    void applyForce(const glm::vec3& f) { for (int i = 0; i < num_particles_; ++i) applyForce(f, i); }
    void setFix(int ind, bool fixed);

#ifdef HAS_CUDA
    void applyAccelerationCUDA(const glm::vec3& a);
#endif

    void addVertex(const glm::vec3& pos, float r, float m);
    void addFace(const Face& face);

    
    void computeNormals();
    void wettingDiffusion(float dt);
    
#ifdef HAS_CUDA
    void computeNormalsCUDA();
#endif

    std::vector<Particle>& getParticles() { return particles_; }
    std::vector<glm::vec3>& getFaceNormals() { return face_norms_; }
    std::vector<glm::vec3>& getVertexNormals() { return vertex_norms_; }
    inline const int getNumParticles() const { return num_particles_; }
    inline const int getNumFaces() const { return num_faces_; }
    inline const int getNumSprings() const { return num_springs_; }
    
#ifdef HAS_CUDA
    Particle* getParticlesCUDA() { return cuda_particles_; }
    Face* getFacesCUDA() { return cuda_faces_; }
    glm::vec3* getFaceNormalsCUDA() { return cuda_face_norms_; }
    glm::vec3* getVertexNormalsCUDA() { return cuda_vertex_norms_; }
#endif
    Mesh getMesh();
    std::vector<float> getWettings();

    virtual std::vector<glm::vec3>& getOldPositions() { assert(false); }

protected:
    const bool cuda_enabled_;
    int cuda_block_size_;

    int num_x_, num_z_;
    int num_particles_, num_springs_, num_faces_;
    std::vector<Particle> particles_;
    std::vector<bool> fixed_;
    std::vector<Spring> springs_;
    std::vector<Face> faces_;
    std::vector<glm::vec3> prev_acceleration_;
    std::vector<glm::vec3> force_buffer_;

    std::vector<glm::vec3> face_norms_, vertex_norms_;
#ifdef HAS_CUDA
    Particle *cuda_particles_;
    Face *cuda_faces_;
    glm::vec3 *cuda_face_norms_, *cuda_vertex_norms_;
    bool *cuda_fixed_;
#endif

    float damping_;
    SpatialHash hash_table_;

    float wetting_speed_;
};

#ifdef HAS_CUDA
__global__ void ClothcollisionWithSphereTask(Particle* particles, int num_particles, glm::vec3 center, float radius, float dt);
__global__ void ClothcomputeFaceNormalsTask(Particle* particles, Face* faces, glm::vec3* vertex_norms, glm::vec3* face_norms, int num_particles, int num_faces);
__global__ void ClothnormalizeVertexNormalsTask(glm::vec3* vertex_norms, int num_particles);
__global__ void ClothapplyAccelerationTask(Particle* particles, bool* fixed, int num_particles, glm::vec3 a);
#endif

class XPBDCloth : public Cloth {
public:
    XPBDCloth(const std::string& path, const YAML::Node& config, const YAML::Node& cuda, float kernel_radius, int hash_table_size);
    ~XPBDCloth();
    std::string getType() const override { return "XPBD"; }

    void update(float dt) override;

    void addDistanceConstraint(int p1, int p2, float stiffness);
    void addBendingConstraint(int p1, int p2, int p3, float stiffness);

    void solveDistanceConstraint(float dt);
    void solveBendingConstraint(float dt);

#ifdef HAS_CUDA
    void solveDistanceConstraintCUDA(float dt);
    void solveBendingConstraintCUDA(float dt);
    void updateCUDA(float dt);
#endif

    std::vector<glm::vec3>& getOldPositions() override { return p_; }

#ifdef HAS_CUDA
    glm::vec3* getOldPositionsCUDA() { return cuda_p_; }
#endif

private:
    int num_distance_constraints_, num_bending_constraints_;
    int iters_;

    std::vector<XPBD_DistanceConstraint> distance_constraints_;
    std::vector<XPBD_BendingConstraint> bending_constraints_;

#ifdef HAS_CUDA
    XPBD_DistanceConstraint *cuda_distance_constraints_;
    XPBD_BendingConstraint *cuda_bending_constraints_;
    glm::vec3 *cuda_p_, *cuda_delta_p_;
    float *cuda_w_;
#endif

    std::vector<glm::vec3> p_, delta_p_;
    std::vector<float> w_;
};

#ifdef HAS_CUDA
__global__ void XPBDsolveDistanceConstraintTask(XPBD_DistanceConstraint* distance_constraints_, Particle* particles_, int num_distance_constraints_, float* cuda_w_, glm::vec3 *cuda_delta_p_, float dt);
__global__ void XPBDupdateTask1(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt);
__global__ void XPBDupdateTask2(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt);
__global__ void XPBDupdateTask3(Particle* cuda_particles_, glm::vec3* cuda_delta_p_, glm::vec3* cuda_p_, int num_particles_, float damping_, int iters_, float dt);
#endif

#endif