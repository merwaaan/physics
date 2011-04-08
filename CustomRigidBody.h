#ifndef CUSTOMRIGIDBODY_H
#define CUSTOMRIGIDBODY_H

#include <vector>

#include "Geometry.h"
#include "RigidBody.h"

struct Structure
{
  std::vector<CustomVertex> vertices;
  std::vector<CustomPolygon> polygons;

	std::vector<Edge> getEdges() const;
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
 
    std::vector<Contact> isCollidingWith(RigidBody* rb_p, double dt);

    std::vector<Contact> isCollidingWith(Sphere* s_p, double dt);

    std::vector<Contact> isCollidingWith(CustomRigidBody* rb_p, double dt);
    bool findSeparationPlane(CustomRigidBody* rb_p);
    std::vector<Contact> resolveInterPenetration(CustomRigidBody* rb_p, double dt);
    Vector3 getSupportPoint(Vector3 direction);

    CustomVertex* getVertexById_p(int id);
    int getPolyCount();
};

#endif

