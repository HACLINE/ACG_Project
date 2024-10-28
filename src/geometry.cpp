#include "geometry.h"
#include <iostream>
#include <map>

glm::mat3 cross_matrix(const glm::vec3& v) {
    return glm::mat3(0.0f, -v.z, v.y, v.z, 0.0f, -v.x, -v.y, v.x, 0.0f);
}

glm::mat3 rotation_matrix(const glm::quat& orientation) {
    return glm::mat3_cast(orientation);
}

glm::vec3 transform(const glm::vec3& position, const glm::vec3& translation, const glm::quat& orientation) {
    glm::mat3 rotation = rotation_matrix(orientation);
    return rotation * position + translation;
}

Mesh mesh_translation(Mesh mesh, const glm::vec3& translation, const glm::quat& orientation) {
    for (auto& vertex : mesh.vertices) {
        vertex.position = transform(vertex.position, translation, orientation);
    }
    return mesh;
}

struct Edge {
  int u;
  int v;
  Edge(int _u, int _v) {
    if (_u > _v)
      std::swap(_u, _v);
    u = _u;
    v = _v;
  }
  bool operator<(const Edge &edge) const {
    if (u < edge.u)
      return true;
    else if (u > edge.u)
      return false;
    return v < edge.v;
  }
};

struct EdgeInfo {
  int id;
  int face_num;
  int f[2];
  glm::vec3 odd_vertice;
};

struct VerticeInfo {
  int num;
  std::vector<int> adj_vertices;
  glm::vec3 even_vertice;
};

void mesh_subdivision(Mesh& mesh) {
    std::map<glm::vec3, int, vec3compare> index_map;
    std::vector<glm::vec3> vertices_new;
    std::vector<glm::ivec3> faces_new;
    auto get_vertex_index = [&](const glm::vec3 &v) {
      if (!index_map.count(v)) {
        index_map[v] = int(vertices_new.size());
        vertices_new.push_back(v);
      }
      return index_map.at(v);
    };
    auto insert_triangle = [&](const glm::vec3 v0, const glm::vec3 v1,
                               const glm::vec3 v2) {
      faces_new.emplace_back(get_vertex_index(v0), get_vertex_index(v1),
                             get_vertex_index(v2));
    };
    std::map<Edge, EdgeInfo> edge_map;
    std::vector<VerticeInfo> vertice_info;
    int vertice_count = mesh.vertices.size();
    int face_count = mesh.faces.size();
    int edge_tot = 0, face_tot = 0;

    auto get_edge_info = [&](const Edge &v) {
      if (!edge_map.count(v)) {
        edge_map[v] = EdgeInfo{edge_tot++, 0, {-1, -1}, glm::vec3{0.0f}};
      }
      return edge_map.at(v);
    };

    auto add_adj_vertice = [&](const int &v1, const int &v2) {
      vertice_info[v1].num ++;
      vertice_info[v1].adj_vertices.push_back(v2);
    };

    auto add_edge_info = [&](const Edge &v, const int &face_id) {
      auto e = get_edge_info(v);
      assert(e.face_num < 2);
      e.f[e.face_num] = face_id;
      e.face_num++;
      if (e.face_num == 1) {
        add_adj_vertice(v.u, v.v);
        add_adj_vertice(v.v, v.u);
      }
      edge_map[v] = e;
    };

    auto find_beta = [&](const int &cnt) { // Warren's
      assert(cnt >= 0);
      if (cnt <= 2){
        return 0.0;
      }
      else if (cnt == 3) {
        return 3.0 / 16.0;
      }
      else {
        return 3.0 / (8.0 * cnt);
      }
    };

    for (int _ = 0; _ < vertice_count; _ ++) {
      vertice_info.push_back(VerticeInfo{0, std::vector<int>(), glm::vec3{0.0f}});
    }
    for (auto &face : mesh.faces) {
      auto i0 = face.v1, i1 = face.v2, i2 = face.v3;
      add_edge_info(Edge(i0, i1), face_tot);
      add_edge_info(Edge(i1, i2), face_tot);
      add_edge_info(Edge(i2, i0), face_tot);
      face_tot++;
    }
    for (int _ = 0; _ < vertice_count; _ ++) {
      int cnt = vertice_info[_].num;
      float beta = find_beta(cnt);
      for (int i = 0; i < cnt; i ++) {
        vertice_info[_].even_vertice += mesh.vertices[vertice_info[_].adj_vertices[i]].position * beta;
      }
      vertice_info[_].even_vertice += mesh.vertices[_].position * (float)(1.0 - beta * cnt);
    }
    for (auto &edge : edge_map) {
      if (edge.second.face_num == 1) {
        edge.second.odd_vertice = (float)0.5 * (mesh.vertices[edge.first.u].position + mesh.vertices[edge.first.v].position);
      }
      else if (edge.second.face_num == 2){
        edge.second.odd_vertice = (float)0.375 * (mesh.vertices[edge.first.u].position + mesh.vertices[edge.first.v].position);
      }
      else {
        assert(false);
      }
    }
    for (auto &face : mesh.faces) {
      auto i0 = face.v1, i1 = face.v2, i2 = face.v3;
      auto e0 = get_edge_info(Edge(i0, i1));
      auto e1 = get_edge_info(Edge(i1, i2));
      auto e2 = get_edge_info(Edge(i2, i0));
      if (e0.face_num == 2) {
        e0.odd_vertice += (float)(0.125) * mesh.vertices[i2].position;
        edge_map[Edge(i0, i1)] = e0;
      }
      if (e1.face_num == 2) {
        e1.odd_vertice += (float)(0.125) * mesh.vertices[i0].position;
        edge_map[Edge(i1, i2)] = e1;
      }
      if (e2.face_num == 2) {
        e2.odd_vertice += (float)(0.125) * mesh.vertices[i1].position;
        edge_map[Edge(i2, i0)] = e2;
      }
    }

    for (auto &face : mesh.faces) {
      auto i0 = face.v1, i1 = face.v2, i2 = face.v3;
      auto e0 = get_edge_info(Edge(i0, i1));
      auto e1 = get_edge_info(Edge(i1, i2));
      auto e2 = get_edge_info(Edge(i2, i0));
      auto _i0 = vertice_info[i0].even_vertice;
      auto _i1 = vertice_info[i1].even_vertice;
      auto _i2 = vertice_info[i2].even_vertice;
      auto _v0 = e0.odd_vertice;
      auto _v1 = e1.odd_vertice;
      auto _v2 = e2.odd_vertice;
      insert_triangle(_i0, _v0, _v2);
      insert_triangle(_i1, _v1, _v0);
      insert_triangle(_i2, _v2, _v1);
      insert_triangle(_v0, _v1, _v2);
    }
    /*****************************/
    mesh.vertices.clear();
    mesh.faces.clear();
    for (auto &v : vertices_new) {
      mesh.vertices.push_back(Vertex{v});
    }
    for (auto &f : faces_new) {
      mesh.faces.push_back(Face{f.x, f.y, f.z});
    }
  }

glm::vec3 get_spring_force(const Spring& spring, const glm::vec3& dP, const glm::vec3& dV) {
    float dis = glm::length(dP);
    float f_s = - spring.k_s * (dis - spring.rest_length);
    float f_d = 0;//spring.k_d * (glm::dot(dP, dV) / dis);
    // std::cout << "f_s: " << f_s << ", f_d: " << f_d << std::endl;
    // if (abs(f_s) > 10) exit(0);
    glm::vec3 force = (f_s + f_d) * glm::normalize(dP);
    return force;
}