#ifndef RENDER_H
#define RENDER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "geometry.h"
#include "simulation.h"
#include "rigid_body.h"
#include "fluid.h"


// Renderer class
class Renderer {
public:
    Renderer(YAML::Node config, std::string figuresPath);
    void initializeOpenGL(YAML::Node config);
    void initializeReconstruction(YAML::Node config);

    void renderMesh(const Mesh&, glm::vec3, std::vector<float>);
    void renderParticles(const std::vector<Particle>&);

    void renderRigidbody(Rigidbody*);
    void renderFluid(Fluid*, int);
    void renderCloth(Cloth*);
    void renderTriangle(Triangle*);
    void renderSphere(Sphere*);

    void renderSimulation(const Simulation&, int);
    void renderObject(const RenderObject&);

    void renderFloor();

    std::vector<float> particlesToDensityField(const std::vector<Particle>& particles, const glm::ivec3& gridResolution, float gridSpacing);

private:
    YAML::Node config_;
    std::string reconstruction_args_, figuresPath_;
    bool enable_gl_;
};

#endif