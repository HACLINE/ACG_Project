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

const static int MAX_CLOTH_SIZE = 100;
const static int START_BUTTON_SIZE = 100;
static bool control_fix[MAX_CLOTH_SIZE][MAX_CLOTH_SIZE];

static PanelInfo *control_panel = nullptr, *last_panel = nullptr;

static void mouse(const int button, const int state, const int x, const int y);
void controlPanelThread(int argc, char* argv[]);
void assignControlPanel(PanelInfo* panel_info);
PanelInfo* getControlPanel();
void assignControlFix(int x, int y, bool fix);
bool getControlFix(int x, int y);

#endif