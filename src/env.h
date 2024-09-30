#ifndef ENV_H
#define ENV_H

#include <vector>
#include "geometry.h"

class Simulation {
    public:
        void initialize(const std::vector<Vertex>& solidVertices, const std::vector<Face>& solidFaces,
                        const std::vector<Particle>& liquidParticles);

        void update(float dt);

        void getSolidMesh(std::vector<Vertex>& vertices, std::vector<Face>& faces) const;

        void getLiquidParticles(std::vector<Particle>& particles) const;
    
    private:
        std::vector<Vertex> solidVertices_;
        std::vector<Face> solidFaces_;
        std::vector<Particle> liquidParticles_;
};

#endif