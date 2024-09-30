#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <string>
#include "geometry.h"

class Rigidbody {
public:
    Rigidbody();
    Rigidbody(const Mesh&);
    Rigidbody(const std::string& filename);
    ~Rigidbody();

    inline const Mesh& getMesh() const { return mesh_; }

private:
    Mesh mesh_;
};

#endif