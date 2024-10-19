#include <yaml-cpp/yaml.h>
#include <GLUT/glut.h>
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

#include "yaml.h"
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

std::queue<RenderObject> render_queue;
std::mutex render_mutex;
std::condition_variable render_cv;
std::atomic<bool> done(false);

void renderThread(Renderer& renderer, const YAML::Node& config, Simulation& simulation, int FPS, int SPS, int VIDEO_LENGTH) {
    renderer.initializeOpenGL(config["render"]);
    int frame = 0;
    auto last = std::chrono::high_resolution_clock::now();

    while (!done) {
        std::unique_lock<std::mutex> lock(render_mutex);
        render_cv.wait(lock, [last, FPS] {
            return (!render_queue.empty() || done) && 
                   (std::chrono::high_resolution_clock::now() - last > std::chrono::milliseconds(static_cast<int>(1000.0f / FPS)));
        });

        while (!render_queue.empty()) {
            RenderObject render_object = render_queue.front();
            render_queue.pop();
            lock.unlock();

            std::cout << "[Render] Rendering frame " << frame << std::endl;

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // renderer.renderFloor();
            renderer.renderObject(render_object);

            std::string file_name = "./figures/frame_" + intToString(frame, 6) + ".png";
            saveFrame(file_name, config["render"]["windowsize"][0].as<int>(), config["render"]["windowsize"][1].as<int>());

            renderer.swapBuffers();

            GLenum err;
            while ((err = glGetError()) != GL_NO_ERROR) {
                std::cerr << "[Error] OpenGL error: " << err << std::endl;
            }

            lock.lock();
            ++frame;
            last = std::chrono::high_resolution_clock::now();
        }
    }
    generateVideo(config["video"]);
}

/*
./main --config config.yaml
*/
int main(int argc, char* argv[]) {

    std::cout << "[main] Starting simulation" << std::endl;

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

    if (!config["cuda"]["enabled"].as<bool>()) {
        std::cout << "[CUDA] CUDA disabled" << std::endl;
    }
#ifdef HAS_CUDA
    else {
        std::cout << "[CUDA] CUDA enabled" << std::endl;
    }
#else
    else {
        std::cout << "[CUDA WARNING] No CUDA support, automatically disabled" << std::endl;
        config["cuda"]["enabled"] = false;
    }
#endif

    Renderer renderer(config["render"]);
    YAML::Node load_config = config["load"];
    load_config["cwd"] = config["cwd"];
    Simulation simulation(load_config, config["simulation"], config["cuda"]);

    int FPS = config["video"]["fps"].as<int>();
    int SPS = config["video"]["sps"].as<int>();
    assert(SPS % FPS == 0);
    float VIDEO_LENGTH = config["video"]["length"].as<float>();

    std::cout << "[Generate] Generating images into ./figures ..." << std::endl;

    std::thread render_thread(renderThread, std::ref(renderer), std::ref(config), std::ref(simulation), FPS, SPS, VIDEO_LENGTH);

    for (int _ = 0; _ < SPS * VIDEO_LENGTH; _++) {
        std::cout << "[Progress] Step " << _ << " / " << SPS * VIDEO_LENGTH << std::endl;
        std::cout.flush();
        simulation.update(1.0f / SPS);

        if (_ % (SPS / FPS) == 0) {
            std::unique_lock<std::mutex> lock(render_mutex);
            render_queue.push(simulation.getRenderObject());
            lock.unlock();
            render_cv.notify_one();
        }
    }

    done = true;
    render_cv.notify_all();
    render_thread.join();


    return 0;
}