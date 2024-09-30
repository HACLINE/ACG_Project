#include "env.h"
#include "geometry.h"

void Simulation::initialize(const std::vector<Vertex>& solidVertices, const std::vector<Face>& solidFaces,
                            const std::vector<Particle>& liquidParticles) {
    solidVertices_ = solidVertices;
    solidFaces_ = solidFaces;
    liquidParticles_ = liquidParticles;
}

void Simulation::update(float dt) {
    return ;
}

void Simulation::getSolidMesh(std::vector<Vertex>& vertices, std::vector<Face>& faces) const {
    vertices = solidVertices_;
    faces = solidFaces_;
}

void Simulation::getLiquidParticles(std::vector<Particle>& particles) const {
    particles = liquidParticles_;
}