#include "yaml.h"
#include <yaml-cpp/yaml.h>
#include <GLUT/glut.h>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include "render.h"
#include "env.h"
#include "genvideo.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void saveFrame(const std::string& filename, int width, int height) {
    std::vector<unsigned char> buffer(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename.c_str(), width, height, 3, buffer.data(), width * 3);
}

std::string intToString(int x, int len) {
    std::string s = std::to_string(x);
    while (s.size() < len) {
        s = "0" + s;
    }
    return s;
}

/*
./main --config config.yaml
*/
int main(int argc, char* argv[]) {

    std::cout << "[main] Starting simulation ..." << std::endl;

    // make args into a map
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
        assert(argv[i][0] == '-' && argv[i][1] == '-');
        argv[i] += 2;
        args[argv[i]] = argv[i + 1];
    }

    // load config
    assert(args.find("config") != args.end());
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        perror("getcwd() error");
        return 1;
    }
    YAML::Node config = yaml_solver(args["config"], std::string(cwd));

    Renderer renderer(config["render"]);
    YAML::Node load_config = config["load"];
    load_config["cwd"] = config["cwd"];

    std::cout << "[main] Config loaded" << std::endl;

    Simulation simulation(load_config, config["simulation"], config["cuda"]);

    std::cout << "[main] Simulation loaded: " << simulation.getNumRigidbodies() << " rigid bodies, " << simulation.getNumFluids() << " fluids, " << simulation.getNumCloths() << " cloths, " << simulation.getNumWalls() << " walls." << std::endl;

    int FPS = config["video"]["fps"].as<int>();
    int SPS = config["video"]["sps"].as<int>();
    assert(SPS % FPS == 0);
    float VIDEO_LENGTH = config["video"]["length"].as<float>();

    std::cout << "[Generate] Generating images into ./figures ... " << std::endl;

    float barcount = 0, bartotal = 30;

    for (int _ = 0; _ < SPS * VIDEO_LENGTH; _++) {
        
        while ((barcount + 1.0) * SPS * VIDEO_LENGTH <= bartotal * _ - 0.01) {
            barcount++;
            std::cout << "#";
            if (int(barcount) % 5 == 0) std::cout<<barcount;
            std::cout.flush();
        }

        if (_ == SPS * 8.0) { // loose the first particle in 3 seconds
            Cloth* cloth = simulation.getCloth(0);
            cloth->setFix(0, false);
            cloth->setFix(19, false);
        }

        simulation.update(1.0f / SPS);
        if (_ % (SPS / FPS) != 0) {
            continue;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // renderer.renderFloor();
        renderer.renderSimulation(simulation);

        std::string file_name = "./figures/frame_" + intToString(_, 6) + ".png";
        saveFrame(file_name, config["render"]["windowsize"][0].as<int>(), config["render"]["windowsize"][1].as<int>());

        renderer.swapBuffers();

        GLenum err;
        while ((err = glGetError()) != GL_NO_ERROR) {
            std::cerr << std::endl << "[Error] OpenGL error: " << err << std::endl;
        }
    }

    std::cout << std::endl;

    generateVideo(config["video"]);

    return 0;
}