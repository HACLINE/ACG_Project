#include "render.h"
#include <iostream>
#include <fstream>
#ifdef __linux__
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif

#include "utils.h"
#include "marching_cube.h"

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif
#include "stb_image_write.h"

void saveFrame(const std::string& filename, int width, int height) {
    std::vector<unsigned char> buffer(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename.c_str(), width, height, 3, buffer.data(), width * 3);
}

void saveSphereToFile(const Sphere* sphere, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << sphere->center.x << " " << sphere->center.y << " " << sphere->center.z << " " << sphere->radius << std::endl;
    file.close();
}

Renderer::Renderer(YAML::Node config, std::string figurePath) : config_(config), figuresPath_(figurePath) {
    enable_gl_ = config["enable_gl"].as<bool>();
    if (enable_gl_) {
        initializeOpenGL(config);
    } 
    initializeReconstruction(config);
}

void Renderer::initializeOpenGL(YAML::Node config) {
    int argc = config["init"]["argc"].as<int>();
    char** argv = new char*[argc];
    for (int i = 0; i < argc; i++) {
        argv[i] = new char[config["init"]["argv"][i].as<std::string>().size() + 1];
        strcpy(argv[i], config["init"]["argv"][i].as<std::string>().c_str());
    }
    glutInit(&argc, argv);
    for (int i = 0; i < argc; i++) {
        delete[] argv[i];
    }
    delete[] argv;
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(config["windowsize"][0].as<int>(), config["windowsize"][1].as<int>());
    glutCreateWindow(config["title"].as<std::string>().c_str());
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glViewport(config["viewport"][0].as<int>(), config["viewport"][1].as<int>(), config["viewport"][2].as<int>(), config["viewport"][3].as<int>());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(config["perspective"]["fovy"].as<float>(), config["perspective"]["aspect"].as<float>(), config["perspective"]["znear"].as<float>(), config["perspective"]["zfar"].as<float>());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClearColor(config["clearcolor"][0].as<float>(), config["clearcolor"][1].as<float>(), config["clearcolor"][2].as<float>(), config["clearcolor"][3].as<float>());
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    gluLookAt(config["camera"]["eye"][0].as<float>(), config["camera"]["eye"][1].as<float>(), config["camera"]["eye"][2].as<float>(), config["camera"]["center"][0].as<float>(), config["camera"]["center"][1].as<float>(), config["camera"]["center"][2].as<float>(), config["camera"]["up"][0].as<float>(), config["camera"]["up"][1].as<float>(), config["camera"]["up"][2].as<float>());
    
    GLfloat light_position[] = {config["light"]["position"][0].as<float>(), config["light"]["position"][1].as<float>(), config["light"]["position"][2].as<float>(), config["light"]["position"][3].as<float>()};
    GLfloat light_ambient[] = {config["light"]["ambient"][0].as<float>(), config["light"]["ambient"][1].as<float>(), config["light"]["ambient"][2].as<float>(), config["light"]["ambient"][3].as<float>() };
    GLfloat light_diffuse[] = {config["light"]["diffuse"][0].as<float>(), config["light"]["diffuse"][1].as<float>(), config["light"]["diffuse"][2].as<float>(), config["light"]["diffuse"][3].as<float>() };
    GLfloat light_specular[] = {config["light"]["specular"][0].as<float>(), config["light"]["specular"][1].as<float>(), config["light"]["specular"][2].as<float>(), config["light"]["specular"][3].as<float>() };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void Renderer::initializeReconstruction(YAML::Node config) {
    reconstruction_args_ = "-q ";
    reconstruction_args_ += "-r=" + config["reconstruction"]["radius"].as<std::string>() + " ";
    reconstruction_args_ += "-l=" + config["reconstruction"]["laplacian"].as<std::string>() + " ";
    reconstruction_args_ += "-c=" + config["reconstruction"]["curvature"].as<std::string>() + " ";
    reconstruction_args_ += "-t=" + config["reconstruction"]["surfacetension"].as<std::string>() + " ";
    if (config["reconstruction"]["subdomain-grid"].as<bool>()) {
        reconstruction_args_ += "--subdomain-grid=on ";
    } else {
        reconstruction_args_ += "--subdomain-grid=off ";
    }
    if (config["reconstruction"]["mesh-cleanup"].as<bool>()) {
        reconstruction_args_ += "--mesh-cleanup=on ";
    } else {
        reconstruction_args_ += "--mesh-cleanup=off ";
    }
    if (config["reconstruction"]["mesh-smoothing-weights"].as<bool>()) {
        reconstruction_args_ += "--mesh-smoothing-weights=on ";
    } else {
        reconstruction_args_ += "--mesh-smoothing-weights=off ";
    }
    reconstruction_args_ += "--mesh-smoothing-iters=" + config["reconstruction"]["mesh-smoothing-iters"].as<std::string>() + " ";
    if (config["reconstruction"]["normals"].as<bool>()) {
        reconstruction_args_ += "--normals=on ";
    } else {
        reconstruction_args_ += "--normals=off ";
    }
    reconstruction_args_ += "--normals-smoothing-iters=" + config["reconstruction"]["normals-smoothing-iters"].as<std::string>() + " ";
}

void Renderer::renderMesh(const Mesh& mesh, glm::vec3 color = glm::vec3(0.0f, 0.8f, 0.2f), std::vector<float> wettings = {}) {
    auto interpolate_color = [](float w) {
        const float MIN = 0.5f, MAX = 1.0f;
        const float THRES1 = 0.3f, THRES2 = 50.0f;
        if (w < THRES1) {
            return MAX;
        } else {
            return MIN + THRES1 * (MAX - MIN) / w;
        }
    };

    glBegin(GL_TRIANGLES);
    if (wettings.empty()) {
        for (const auto& face : mesh.faces) {
            const glm::vec3& v1 = mesh.vertices[face.v1].position;
            const glm::vec3& v2 = mesh.vertices[face.v2].position;
            const glm::vec3& v3 = mesh.vertices[face.v3].position;

            glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            glNormal3f(normal.x, normal.y, normal.z);

            glColor3f(color.x, color.y, color.z);
            glVertex3f(v1.x, v1.y, v1.z);
            glVertex3f(v2.x, v2.y, v2.z);
            glVertex3f(v3.x, v3.y, v3.z);
        }
    } else {
        assert(wettings.size() == mesh.vertices.size());
        for (const auto& face : mesh.faces) {
            const glm::vec3& v1 = mesh.vertices[face.v1].position;
            const glm::vec3& v2 = mesh.vertices[face.v2].position;
            const glm::vec3& v3 = mesh.vertices[face.v3].position;
            float w1 = interpolate_color(wettings[face.v1]);
            float w2 = interpolate_color(wettings[face.v2]);
            float w3 = interpolate_color(wettings[face.v3]);

            glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            glNormal3f(normal.x, normal.y, normal.z);

            glColor3f(w1 * color.x, w1 * color.y, w1 * color.z);
            glVertex3f(v1.x, v1.y, v1.z);
            glColor3f(w2 * color.x, w2 * color.y, w2 * color.z);
            glVertex3f(v2.x, v2.y, v2.z);
            glColor3f(w3 * color.x, w3 * color.y, w3 * color.z);
            glVertex3f(v3.x, v3.y, v3.z);
        }
    }
    glEnd();
}

void Renderer::renderParticles(const std::vector<Particle>& particles) {
    for (const auto& particle : particles) {
        glPushMatrix();
        glTranslatef(particle.position.x, particle.position.y, particle.position.z);
        glColor3f(1.0f, 0.0f, 0.0f);
        glutSolidSphere(particle.radius, 20, 20);
        glPopMatrix();
    }
}

void Renderer::renderFloor() {
    // glBegin(GL_QUADS);
    // glColor3f(0.8f, 0.8f, 0.8f);
    // glVertex3f(-100.0f, -10.0f, -100.0f);
    // glVertex3f( 100.0f, -10.0f, -100.0f);
    // glVertex3f( 100.0f, -10.0f,  100.0f);
    // glVertex3f(-100.0f, -10.0f,  100.0f);
    // glEnd();
}

void Renderer::renderRigidbody(Rigidbody* rigidbody) {
    renderMesh(rigidbody->getCurrentMesh());
}

#define SPLASH_SURF 0
#define MARCHING_CUBES 1
#define POINT_CLOUD 2
#define VOXEL 3

void Renderer::renderFluid(Fluid* fluid, int method = SPLASH_SURF) {
    if (method == POINT_CLOUD) {
        renderParticles(fluid->getParticles());
        return ;
    }

    const auto& particles = fluid->getParticles();

    if (method == VOXEL) {
        glm::ivec3 gridResolution(40, 40, 40);
        float gridSpacing = 1.0f / 15.0f;
        std::vector<float> densityField = particlesToDensityField(particles, gridResolution, gridSpacing);
        for (int z = 0; z < gridResolution.z; ++z) {
            for (int y = 0; y < gridResolution.y; ++y) {
                for (int x = 0; x < gridResolution.x; ++x) {
                    int index = x + y * gridResolution.x + z * gridResolution.x * gridResolution.y;
                    if (densityField[index] > 0.0f) {
                        glPushMatrix();
                        glTranslatef(x * gridSpacing - 1.0f, y * gridSpacing - 1.0f, z * gridSpacing - 1.0f);
                        glColor3f(0.0f, 0.0f, 1.0f);
                        glutSolidCube(gridSpacing);
                        glPopMatrix();
                    }
                }
            }
        }
    } else if (method == MARCHING_CUBES) {
        glm::ivec3 gridResolution(40, 40, 40);
        float gridSpacing = 1.0f / 15.0f;
        std::vector<float> densityField = particlesToDensityField(particles, gridResolution, gridSpacing);
        std::vector<Vertex> vertices;
        std::vector<Face> indices;
        MarchingCubes::generateSurface(densityField, gridResolution, gridSpacing, vertices, indices);
        Mesh mesh{vertices, indices};
        mesh_subdivision(mesh);

        renderMesh(mesh);
    } else if (method == SPLASH_SURF) {
        saveParticlesToPLY(particles, "./.cache/tmp/particles.ply");

        std::string command = "splashsurf reconstruct ./.cache/tmp/particles.ply -o ./.cache/tmp/mesh.obj " + reconstruction_args_;

        system(command.c_str());

        Mesh mesh = loadMeshFromOBJ("./.cache/tmp/mesh.obj");

        renderMesh(mesh);
    }
}

void Renderer::renderCloth(Cloth* cloth) {
    renderMesh(cloth->getMesh(), glm::vec3(0.8f, 0.0f, 0.0f), cloth->getWettings());
}

void Renderer::renderTriangle(Triangle* triangle) {
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.8f);
    glVertex3f(triangle->v1.x, triangle->v1.y, triangle->v1.z);
    glVertex3f(triangle->v2.x, triangle->v2.y, triangle->v2.z);
    glVertex3f(triangle->v3.x, triangle->v3.y, triangle->v3.z);
    glEnd();
}

void Renderer::renderSphere(Sphere* sphere) {
    glPushMatrix();
    glTranslatef(sphere->center.x, sphere->center.y, sphere->center.z);
    glColor3f(0.0f, 0.0f, 0.6f);
    glutSolidSphere(sphere->radius - 0.01, 20, 20);
    glPopMatrix();
}

void Renderer::renderSimulation(const Simulation& simulation, int frame) {
    if (enable_gl_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for (int i = 0; i < simulation.getNumRigidbodies(); i++) {
            renderRigidbody(simulation.getRigidbody(i));
        }
        for (int i = 0; i < simulation.getNumFluids(); i++) {
            renderFluid(simulation.getFluid(i));
        }
        for (int i = 0; i < simulation.getNumCloths(); i++) {
            renderCloth(simulation.getCloth(i));
        }
        for (int i = 0; i < simulation.getNumWalls(); i++) {
            renderTriangle(simulation.getWall(i));
        }
        for (int i = 0; i < simulation.getNumSpheres(); i++) {
            renderSphere(simulation.getSphere(i));
        }    
        
        std::string file_name = figuresPath_ + "/frame_" + intToString(frame, 6) + ".png";
        saveFrame(file_name, config_["windowsize"][0].as<int>(), config_["windowsize"][1].as<int>());

        glutSwapBuffers();
    }
    std::string subdir = "./.cache/frame_" + intToString(frame, 6);
    std::string command = "mkdir " + subdir + " && mkdir " + subdir + "/rigid_bodies && mkdir " + subdir + "/fluids && mkdir " + subdir + "/cloths && mkdir " + subdir + "/spheres";
    system(command.c_str());

    for (int i = 0; i < simulation.getNumRigidbodies(); i++) {
        saveMeshToOBJ(simulation.getRigidbody(i)->getCurrentMesh(), subdir + "/rigid_bodies/" + intToString(i, 2) + ".obj");
    }

    for (int i = 0; i < simulation.getNumFluids(); i++) {
        saveParticlesToPLY(simulation.getFluid(i)->getParticles(), subdir + "/fluids/" + intToString(i, 2) + ".ply");
        command = "splashsurf reconstruct " + subdir + "/fluids/" + intToString(i, 2) + ".ply -o " + subdir + "/fluids/" + intToString(i, 2) + ".obj " + reconstruction_args_ + "&& rm " + subdir + "/fluids/" + intToString(i, 2) + ".ply";
        system(command.c_str());
    }

    for (int i = 0; i < simulation.getNumCloths(); i++) {
        saveMeshToOBJ(simulation.getCloth(i)->getMesh(), subdir + "/cloths/" + intToString(i, 2) + ".obj");
    }

    for (int i = 0; i < simulation.getNumSpheres(); i++) {
        saveSphereToFile(simulation.getSphere(i), subdir + "/spheres/" + intToString(i, 2) + ".sphere");
    }

    // std::cerr << "[Render ERROR] Blender rendering does not support the following settings: " << std::endl;
    // std::cerr << "Rigidbodies: " << simulation.getNumRigidbodies() << std::endl;
    // std::cerr << "Fluids: " << simulation.getNumFluids() << std::endl;
    // std::cerr << "Cloths: " << simulation.getNumCloths() << std::endl;
    // std::cerr << "Stopping simulation." << std::endl;
    // exit(1);
}

void Renderer::renderObject(const RenderObject& object) {
    for (const auto& mesh : object.meshes) {
        renderMesh(mesh);
    }
    for (const auto& particles : object.particles) {
        renderParticles(particles);
    }
}

std::vector<float> Renderer::particlesToDensityField(const std::vector<Particle>& particles, const glm::ivec3& gridResolution, float gridSpacing) {
    std::vector<float> densityField(gridResolution.x * gridResolution.y * gridResolution.z, 0.0f);
    for (const auto& particle : particles) {
        glm::vec3 pos = particle.position + glm::vec3(1.05f, 1.05f, 1.05f);
        glm::ivec3 gridIndex = glm::ivec3(pos / gridSpacing);
        int index = gridIndex.x + gridIndex.y * gridResolution.x + gridIndex.z * gridResolution.x * gridResolution.y;
        if (index < 0 || index >= densityField.size()) {
            std::cerr << "[Render WARNING] Index out of bounds: " << index << std::endl;
            continue;
        }
        densityField[index] += 1.0f;
    }
    return densityField;
}

static void mouse(const int button, const int state, const int x, const int y) {
    // std::cout << "Mouse: " << button << " " << state << " " << x << " " << y << std::endl;
    copyPanelInfo(control_panel, last_panel);
    control_panel->click_x = x;
    control_panel->click_y = control_panel->window_y + START_BUTTON_SIZE - y - 1;
    control_panel->l_click_button |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
    control_panel->r_click_button |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
    control_panel->m_click_button |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

    control_panel->l_click_button &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
    control_panel->r_click_button &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
    control_panel->m_click_button &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

    control_panel->click_down = control_panel->l_click_button || control_panel->r_click_button || control_panel->m_click_button;

    if (control_panel->click_down && !last_panel->click_down) {
        int x = control_panel->click_x;
        int y = control_panel->click_y;
        if (y > control_panel->window_y && y < control_panel->window_y + START_BUTTON_SIZE) {
            control_panel->finished = true; // start button
            return ;
        }
        float dx = float(control_panel->window_x) / control_panel->num_x;
        float dy = float(control_panel->window_y) / control_panel->num_y;
        int i = x / dx;
        int j = y / dy;
        assignControlFix(i, j, !control_fix[i][j]);
    }
}

void display() {
    // std::cout << "display" << std::endl;
    glClear(GL_COLOR_BUFFER_BIT);
    float radius = 3.0f;
    assert(control_panel != nullptr && control_panel->num_x > 0 && control_panel->num_y > 0);
    // std::cout << control_panel->num_x << " " << control_panel->num_y << std::endl;
    for (int i = 0; i < control_panel->num_x; ++i) {
        for (int j = 0; j < control_panel->num_y; ++j) {
            float x = (i + 0.5f) * control_panel->window_x / control_panel->num_x;
            float y = (j + 0.5f) * control_panel->window_y / control_panel->num_y;
            glBegin(GL_POLYGON);
            if (control_fix[i][j]) {
                glColor3f(1.0f, 0.0f, 0.0f); // Red
            } else {
                glColor3f(0.0f, 1.0f, 0.0f); // Green
            }
            // glVertex2f(x, y);
            for (int k = 0; k < 4; ++k) {
                float theta = k * 3.14159f / 2.0f;
                glVertex2f(x + radius * cos(theta), y + radius * sin(theta));
            }
            glEnd();
        }
    }
    // render the start button
    glBegin(GL_POLYGON);
    if (control_panel->finished == true) {
        glColor3f(0.0f, 0.0f, 1.0f); // Blue
    } else {
        glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    }
    float x = control_panel->window_x / 2.0f;
    float y = control_panel->window_y + START_BUTTON_SIZE / 2.0;
    float start_button_radius = START_BUTTON_SIZE / 2.5; // a little smaller
    for (int k = 0; k < 4; ++k) {
        float theta = k * 3.14159f / 2.0f;
        glVertex2f(x + start_button_radius * cos(theta), y + start_button_radius * sin(theta));
    }
    glEnd();
    // std::cout << "end" << std::endl;
    glutSwapBuffers();
}

void timer(int value) {
    glutPostRedisplay();
    glutTimerFunc(200, timer, 0);
}

void controlPanelThread(int argc, char* argv[]) {
    // glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    assert(control_panel != nullptr);
    glutInitWindowSize(control_panel->window_x, control_panel->window_y + START_BUTTON_SIZE);
    glutCreateWindow("Control Panel");
    glutMouseFunc(mouse);
    glutDisplayFunc(display);
    glutTimerFunc(0, timer, 0); // Start the timer
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glLoadIdentity();
    glOrtho(0.0, control_panel->window_x, 0.0, control_panel->window_y + START_BUTTON_SIZE, -1.0, 1.0);
    glutMainLoop();
}

void assignControlPanel(PanelInfo* panel_info) {
    assert(control_panel == nullptr && last_panel == nullptr && panel_info != nullptr);
    assert(panel_info->num_x <= MAX_CLOTH_SIZE && panel_info->num_y <= MAX_CLOTH_SIZE);
    control_panel = panel_info;
    last_panel = new PanelInfo();
    copyPanelInfo(control_panel, last_panel);
}

PanelInfo* getControlPanel() { return control_panel; }

void assignControlFix(int x, int y, bool fix) {
    assert(control_panel != nullptr);
    assert(x >= 0 && x < control_panel->num_x && y >= 0 && y < control_panel->num_y);
    control_fix[x][y] = fix;
}

bool getControlFix(int x, int y) {
    assert(control_panel != nullptr);
    assert(x >= 0 && x < control_panel->num_x && y >= 0 && y < control_panel->num_y);
    return control_fix[x][y];
}