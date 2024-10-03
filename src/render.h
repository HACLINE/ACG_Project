#ifndef RENDER_H
#define RENDER_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include "geometry.h"
#include "env.h"
#include "rigid_body.h"
#include "fluid.h"


// Renderer class
class Renderer {
public:
    Renderer(YAML::Node config);

    void renderMesh(const Mesh&);
    void renderParticles(const std::vector<Particle>&);

    void renderRigidbody(Rigidbody*);
    void renderFluid(Fluid*);

    void renderSimulation(const Simulation&);

    void renderFloor();

    void swapBuffers(); // Swap front and back buffers
};

#endif