#ifndef CUSTOMRIGIDBODY_H
#define CUSTOMRIGIDBODY_H

#include <vector>

#include "RigidBody.h"

struct SeparationPlane
{
  Vector3 point;
  Vector3 normal;

  double getDistanceFromPoint(Vector3 p)
  {
    return -(normal * (point - p)) / normal.length();
  }
};

struct Vertex
{
  int id;
  Vector3 localPosition;
  Vector3 absPosition;
  double mass;
};

struct Polygon
{
  int size;
  Vertex** vertices_p;

  Vector3 getNormal()
  {
    Vector3 v1 = vertices_p[0]->absPosition - vertices_p[1]->absPosition;
    Vector3 v2 = vertices_p[1]->absPosition - vertices_p[2]->absPosition;

    return (v1 ^ v2).normalize();
  }

  SeparationPlane getSeparationPlane()
  {
    SeparationPlane sp;

    sp.point = vertices_p[0]->absPosition;
    sp.normal = getNormal();

    return sp;
  }
};

struct Structure
{
  std::vector<Vertex> vertices;
  std::vector<Polygon> polygons;
};

class CustomRigidBody : public RigidBody
{
  protected:
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

    Vertex* getVertexById_p(int id);
    int getPolyCount();
    Structure* getStructure_p();
};

#endif

