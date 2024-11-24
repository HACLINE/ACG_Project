/*
./main --config [config].yaml

config: see config/ for examples.
*/

#include <yaml-cpp/yaml.h>
#ifdef __linux__
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <map>
#include <chrono>
#include <cstdlib>

#include "yaml.h"
#include "render.h"
#include "simulation.h"
#include "genvideo.h"


std::queue<RenderObject> render_queue;
std::mutex render_mutex;
std::condition_variable render_cv;
std::atomic<bool> done(false);
std::string figuresPath = "/figures";
std::string renderPath = "/render";

int main(int argc, char* argv[]) {

    std::string command = "rm -rf ./.cache && mkdir ./.cache && mkdir ./.cache/tmp";
    system(command.c_str());

    std::cout << "[main] Starting simulation" << std::endl;

    // make args into a map
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        assert(argv[i][0] == '-' && argv[i][1] == '-');
        argv[i] += 2;
        if (i + 1 < argc && argv[i + 1][0] != '-') args[argv[i]] = argv[i + 1], ++i;
        else args[argv[i]] = "";
    }

    // load config
    assert(args.find("config") != args.end());
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        perror("getcwd() error");
        return 1;
    }
    YAML::Node config = yaml_solver(args["config"], std::string(cwd));
    std::string cwd_str = config["cwd"].as<std::string>();
    // if cwd doesn't end up with '/build'
    if (cwd_str.substr(cwd_str.size() - 6) != "/build") cwd_str += "/build";
    figuresPath = cwd_str + figuresPath;
    renderPath = cwd_str + renderPath;

    command = "rm -rf " + figuresPath + " && mkdir " + figuresPath + " && rm -rf " + renderPath + " && mkdir " + renderPath;
    system(command.c_str());

    setenv("CONFIG", args["config"].c_str(), 1);
    setenv("FIGURES", figuresPath.c_str(), 1);
    setenv("RENDER", renderPath.c_str(), 1);

    if (!config["cuda"]["enabled"].as<bool>()) {
        std::cout << "[CUDA] CUDA disabled" << std::endl;
    }
#ifdef HAS_CUDA
    else {
        std::cout << "[CUDA] CUDA enabled" << std::endl;
    }
#else
    else {
        std::cout << "[CUDA WARNING] No CUDA support! Automatically disabled." << std::endl;
        config["cuda"]["enabled"] = false;
    }
#endif

    Renderer renderer(config["render"], figuresPath);
    YAML::Node load_config = config["load"];
    load_config["cwd"] = config["cwd"];
    config["blender"]["render"] = config["render"];
    Simulation simulation(load_config, config["simulation"], config["cuda"], config["blender"]);

    std::cout << "[main] Simulation loaded: " << simulation.getNumRigidbodies() << " rigid bodies, " << simulation.getNumFluids() << " fluids, " << simulation.getNumCloths() << " cloths, " << simulation.getNumWalls() << " walls." << std::endl;

    int FPS = config["video"]["fps"].as<int>();
    int SPS = config["video"]["sps"].as<int>();
    assert(SPS % FPS == 0);
    float VIDEO_LENGTH = config["video"]["length"].as<float>();

    std::cout << "[Generate] Generating images into ./figures ... " << std::endl;

    float barcount = 0, bartotal = 100;

    for (int _ = 0; _ < SPS * VIDEO_LENGTH; _++) {
        while ((barcount + 1.0) * SPS * VIDEO_LENGTH <= bartotal * _ - 0.01) {
            barcount++;
            std::cout << "#";
            if (int(barcount) % 5 == 0) std::cout<<barcount;
            std::cout.flush();
        }

        // if (_ == SPS * 3.0) { // loose the first particle in 3 seconds
        //     Cloth* cloth = simulation.getCloth(0);
        //     cloth->setFix(0, false);
        //     cloth->setFix(19, false);
        // }

        simulation.update(1.0f / SPS);
        if (_ % (SPS / FPS) != 0) {
            continue;
        }

        // renderer.renderFloor();
        renderer.renderSimulation(simulation, _);
        GLenum err;
        while ((err = glGetError()) != GL_NO_ERROR) {
            std::cerr << std::endl << "[Error] OpenGL error: " << err << std::endl;
        }
    }
    std::cout << std::endl;
    if (config["render"]["enable_gl"].as<bool>()) {
        generateVideo(config["video"], figuresPath);
    }
    if (config["blender"]["enabled"].as<bool>()) {
        std::cout << "[Blender] Rendering video ... " << std::endl;
        std::string command = "blender --background --python src/render.py";
        system(command.c_str());
        generateVideo(config["video"], renderPath, "rendered.avi");
    }

    return 0;
}