#ifndef CUSTOMRIGIDBODY_H
#define CUSTOMRIGIDBODY_H

#include <vector>

#include "Geometry.h"
#include "RigidBody.h"

struct Structure
{
  std::vector<CustomVertex> vertices;
  std::vector<CustomPolygon> polygons;
};

struct CachedEdge
{
	int idA;
	int idB;
};

class CustomRigidBody : public RigidBody
{
  public:
    Structure structure;
		std::vector<CachedEdge> cachedEdges;

  public:
    CustomRigidBody();
    ~CustomRigidBody();

    void addVertex(int id, double x, double y, double z, double m);
    void addPolygon(int count, int* ids);
    void prepare();
    void computeCenterOfMass();
    void computeInverseInertiaTensor();
		void computeCachedEdges();
    void computeBoundingBox();

    void integrate(double t);
    void draw();
    void computeVerticesAbsolutePositions();
 
		std::vector<Contact> getContacts(RigidBody* rb_p);
		std::vector<Contact> getContacts(Sphere* s_p);
		std::vector<Contact> getContacts(CustomRigidBody* s_p);
    Vector3 getSupportPoint(Vector3 direction);

    CustomVertex* getVertexById_p(int id);
		std::vector<Edge> getEdges();
};

#endif
