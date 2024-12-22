#ifndef UTILS_H
#define UTILS_H

#include <iostream>

#include "geometry.h"

#include "tiny_obj_loader.h"

std::ostream& operator<<(std::ostream&, const glm::vec3&);

std::ostream& operator<<(std::ostream&, const glm::ivec3&);

std::ostream& operator<<(std::ostream&, const glm::vec4&);

std::ostream& operator<<(std::ostream&, const Particle&);

void saveParticlesToPLY(const std::vector<Particle>& particles, const std::string& filename);

void saveWettingsToCSV(const std::vector<float>& wettings, const std::string& filename);

Mesh loadMeshFromOBJ(const std::string& filename);

void saveMeshToOBJ(const Mesh& mesh, const std::string& filename);

std::string intToString(int x, int len);

struct PanelInfo {
    int window_x = 400, window_y = 400;
    int num_x = 0, num_y = 0;
    int click_x = 0, click_y = 0;
    bool l_click_button = false, r_click_button = false, m_click_button = false, click_down = false;
    bool finished = false;

    PanelInfo() = default;
    PanelInfo(int window_x, int window_y, int num_x, int num_y) : window_x(window_x), window_y(window_y), num_x(num_x), num_y(num_y) { click_x = click_y = 0; l_click_button = r_click_button = m_click_button = click_down = finished = false; }
};

void copyPanelInfo(const PanelInfo* src, PanelInfo* dest);

#endif