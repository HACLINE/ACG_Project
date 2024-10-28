#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

#include <vector>
#include <glm/glm.hpp>
#include <geometry.h>

class MarchingCubes {
public:
    static void generateSurface(const std::vector<float>& densityField, const glm::ivec3& gridResolution, float gridSpacing, std::vector<Vertex>& vertices, std::vector<Face>& indices);
private:
    static int getCubeIndex(const std::array<float, 8>& cubeValues, float isoLevel);
    static glm::vec3 interpolateVertex(const glm::vec3& p1, const glm::vec3& p2, float v1, float v2, float isoLevel);
    static const int edgeTable[256];
    static const int triTable[256][16];
    static const int vertexId[12][2];
    static const int posId[8];
};

#endif // MARCHINGCUBES_H