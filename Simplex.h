#ifndef SIMPLEX_H
#define SIMPLEX_h

#include <vector>

#include "Geometry.h"

class CustomRigidBody;

class Simplex
{
  public:
    std::vector<Vector3> points;

  public:
    Simplex();
    ~Simplex();

    void addSupportPoint(CustomRigidBody* rb_p, Vector3 direction);
    void reduce(Vector3 point);

    Vector3 closestPointFromOrigin() const;

    Vector3 getPoint() const;
    Edge getEdge() const;
    Polygon getPolygon() const;
    CustomRigidBody getTetrahedron() const;
};

#endif
