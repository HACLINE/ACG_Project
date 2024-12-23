#include "utils.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>


#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::ivec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec4& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "Particle: {" << std::endl;
    os << " Position: " << p.position << std::endl;
    os << " Velocity: " << p.velocity << std::endl;
    os << " Acceleration: " << p.acceleration << std::endl;
    os << " Radius: " << p.radius << std::endl;
    os << " Mass: " << p.mass << std::endl;
    os << "}";
    return os;
}

void saveParticlesToPLY(const std::vector<Particle>& particles, const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << particles.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";

    for (const auto& particle : particles) {
        ofs << particle.position.x << " " << particle.position.y << " " << particle.position.z << "\n";
    }

    ofs.close();
}

Mesh loadMeshFromOBJ(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return Mesh();
    }

    Mesh mesh;
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.substr(0, 2) == "v ") {
            std::istringstream iss(line.substr(2));
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            mesh.vertices.push_back(Vertex{vertex});
        } else if (line.substr(0, 3) == "vn ") {
            // ignore normals
        } else if (line.substr(0, 2) == "f ") {
            std::istringstream iss(line.substr(2));
            int v1, v2, v3, v4, v5, v6;
            char slash;
            iss >> v1 >> slash >> slash >> v2 >> v3 >> slash >> slash >> v4 >> v5 >> slash >> slash >> v6;
            // std::cout << v1 << " " << v2 << " " << v3 << " " << v4 << " " << v5 << " " << v6 << std::endl;
            mesh.faces.push_back(Face{v1 - 1, v3 - 1, v5 - 1});
        } else {
            std::cout << "[Render ERROR] Unknown line from mesh.obj: " << line << std::endl;
            exit(1);
        }
    }

    ifs.close();
    return mesh;
}

void saveMeshToOBJ(const Mesh& mesh, const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto& vertex : mesh.vertices) {
        ofs << "v " << vertex.position.x << " " << vertex.position.y << " " << vertex.position.z << "\n";
    }

    for (const auto& face : mesh.faces) {
        ofs << "f " << face.v1 + 1 << " " << face.v2 + 1 << " " << face.v3 + 1 << "\n";
    }

    ofs.close();
}

void saveWettingsToCSV(const std::vector<float>& wettings, const std::string& filename) {
    auto interpolate_color = [](float w) {
        const float MIN = 0.5f, MAX = 1.0f;
        const float THRES1 = 0.3f, THRES2 = 50.0f;
        if (w < THRES1) {
            return MAX;
        } else {
            return MIN + THRES1 * (MAX - MIN) / w;
        }
    };
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    ofs << "wetting\n";
    for (const auto& w : wettings) {
        ofs << interpolate_color(w) << "\n";
    }
}

std::string intToString(int x, int len) {
    std::string s = std::to_string(x);
    while (s.size() < len) {
        s = "0" + s;
    }
    return s;
}

void copyPanelInfo(const PanelInfo* src, PanelInfo* dest) {
    if (src == nullptr || dest == nullptr) {
        return;
    }
    dest->window_x = src->window_x;
    dest->window_y = src->window_y;
    dest->num_x = src->num_x;
    dest->num_y = src->num_y;
    dest->click_x = src->click_x;
    dest->click_y = src->click_y;
    dest->l_click_button = src->l_click_button;
    dest->r_click_button = src->r_click_button;
    dest->m_click_button = src->m_click_button;
    dest->click_down = src->click_down;
    dest->finished = src->finished;
}
