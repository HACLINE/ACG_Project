#ifndef XPBD_CONSTRAINTS_H
#define XPBD_CONSTRAINTS_H

struct XPBD_DistanceConstraint {
    int p1, p2;
    float rest_length;
    float stiffness;
};

struct XPBD_BendingConstraint {
    int p1, p2, p3;
    float rest_length;
    float stiffness;
};

#endif