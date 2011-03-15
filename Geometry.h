#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>

#include "Sphere.h"
#include "Vector3.h"

class CustomRigidBody;
class Sphere;

struct Plane
{
  Vector3 point;
  Vector3 normal;
};

struct Edge
{
  Vector3 a;
  Vector3 b;
};

struct Polygon
{
  int size;
  Vector3* points;
};

struct CustomVertex
{
  int id;
  Vector3 localPosition;
  Vector3 absPosition;
  double mass;
};

struct CustomPolygon
{
  int size;
  CustomVertex** vertices_p;

  std::vector<Edge> getEdges()
  {
    std::vector<Edge> edges;

    for(int i = 0; i < size - 1; ++i)
    {
      Edge s = {vertices_p[i]->absPosition, vertices_p[i + 1]->absPosition};
      edges.push_back(s);
    }

    Edge s = {vertices_p[size - 1]->absPosition, vertices_p[0]->absPosition};
    edges.push_back(s);

    return edges;
  }

  Vector3 getNormal()
  {
    Vector3 v1 = vertices_p[0]->absPosition - vertices_p[1]->absPosition;
    Vector3 v2 = vertices_p[1]->absPosition - vertices_p[2]->absPosition;

    return (v1 ^ v2).normalize();
  }

  Plane getPlane()
  {
    Plane sp;

    sp.point = vertices_p[0]->absPosition;
    sp.normal = getNormal();

    return sp;
  }

  Vector3 getCenter()
  {
    Vector3 center;

    for(int i = 0; i < size; ++i)
      center += vertices_p[i]->absPosition;

    return center / size;
  }
};

namespace Geometry
{
  Vector3 closestPointOfEdge(Vector3 p, Edge edge, double* distance_p = NULL);
  Vector3 closestPointOfPlane(Vector3 p, Plane plane, double* distance_p = NULL);
  Vector3 closestPointOfPolygon(Vector3 p, Polygon polygon, double* distance_p = NULL);
  Vector3 closestPointOfSphere(Vector3 p, Sphere sphere, double* distance_p = NULL);

  Vector3 closestPoints(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p);

  std::vector<CustomVertex> getContactingVertices(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, double tolerance);
};

#endif

