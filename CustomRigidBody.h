#ifndef CUSTOMRIGIDBODY_H
#define CUSTOMRIGIDBODY_H

#include <vector>

#include "Geometry.h"
#include "RigidBody.h"

struct Structure
{
  std::vector<CustomVertex> vertices;
  std::vector<CustomPolygon> polygons;

  Vector3 getSupportPoint(Vector3 direction) const;
};

class CustomRigidBody : public RigidBody
{
  public:
    Structure structure;

  public:
    CustomRigidBody();
    ~CustomRigidBody();

    void addVertex(int id, double x, double y, double z, double m);
    void addPolygon(int count, int* ids);
    void prepare();
    void computeCenterOfMass();
    void computeInverseInertiaTensor();
    void computeBoundingBox();

    void integrate(double t);
    void draw();
    void computeVerticesAbsolutePositions();
 
    Contact* isCollidingWith(RigidBody* rb_p, double dt);

    Contact* isCollidingWith(Sphere* s_p, double dt);

    Contact* isCollidingWith(CustomRigidBody* rb_p, double dt);
    bool findSeparationPlane(CustomRigidBody* rb_p);
    Contact* resolveInterPenetration(CustomRigidBody* rb_p, double dt, double tolerance);

    CustomVertex* getVertexById_p(int id);
    int getPolyCount();
};

#endif

