#ifndef RENDER_H
#define RENDER_H

#include <vector>
#include "geometry.h"

// Renderer class
class Renderer {
public:
    void renderMesh(const std::vector<Vertex>& vertices, const std::vector<Face>& faces);

    void renderParticles(const std::vector<Particle>& particles);

    void renderFloor();

    void swapBuffers(); // Swap front and back buffers
};

#endif