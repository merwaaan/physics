#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>

#include "Vector3.h"
#include "Matrix3.h"

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
};

struct Structure
{
  std::vector<Vertex> vertices;
  std::vector<Polygon> polygons;
};

class RigidBody
{
  public:
    Structure structure;

    double inverseMass;
    
    Vector3 position;
    Vector3 velocity;

    Matrix3 rotation;
    Vector3 angularVelocity;

    Vector3 accumulatedForces;

  public:
    RigidBody();
    ~RigidBody();

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

    void applyForce(Vector3& force);
    void clearAccumulatedForces();
    void integrate(double t);
    void computeVerticesAbsolutePositions();

    void addVertex(int id, double x, double y, double z, double m);
    void addPolygon(int count, int* ids);
    void computeCenterOfMass();

    Vertex* getVertexById_p(int id);
    int getPolyCount();
};

#endif

