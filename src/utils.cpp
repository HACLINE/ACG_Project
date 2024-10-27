#include "utils.h"

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