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

  double inverseMass;

  Matrix3 inverseInertiaTensor;
};

class RigidBody
{
  protected:
    Structure structure;
    
    Vector3 position;
    Vector3 linearMomentum;

    Matrix3 orientation;
    Vector3 angularMomentum;

    Vector3 accumulatedForces;
    Vector3 accumulatedTorques;

    bool fixed;

  public:
    RigidBody();
    ~RigidBody();

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

    void clearAccumulators();
    void applyCenterForce(Vector3 force);
    void applyOffCenterForce(Vector3 force, Vector3 poa);
    void integrate(double t);
    void computeVerticesAbsolutePositions();

    void setPosition(Vector3 position);
    void setOrientation(Matrix3 orientation);
    void setFixed(bool fixed);

    void addVertex(int id, double x, double y, double z, double m);
    void addPolygon(int count, int* ids);
    virtual void prepare();
    virtual void computeCenterOfMass();
    virtual void computeInverseInertiaTensor();

    Vertex* getVertexById_p(int id);
    int getPolyCount();
    Structure* getStructure_p();
};

#endif

