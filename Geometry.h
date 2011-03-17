#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>

#include "Sphere.h"
#include "Vector3.h"

class CustomRigidBody;
class Sphere;

/**
 * EDGE
 *
 * - a : starting point
 * - b : ending point
 */
struct Edge
{
  Vector3 a;
  Vector3 b;
};

/**
 * PLANE
 * 
 * - point : vector to the reference point
 * - normal : normalized vector which origin is point
 */
struct Plane
{
  Vector3 point;
  Vector3 normal;
};

/**
 * POLYGON
 *
 * - size : number of vertices forming the polygon
 * - points : array of points ordered clockwise
 */
struct Polygon
{
  int size;
  std::vector<Vector3> points;
};

/**
 * TRIANGLE
 * - a, b, c : vertices forming the triangle
 */
struct Triangle
{
  Vector3 a;
  Vector3 b;
  Vector3 c;
};

/**
 * TETRAHEDRON
 * - a, b, c : triangles forming the tetrahedron
 */
struct Tetrahedron
{
  Triangle a;
  Triangle b;
  Triangle c;
  Triangle d;
};

/**
 * CUSTOM VERTEX
 *
 * - id : unique key within a custom rigid body
 * - localPosition : vector from the center of mass of the body
 * - absPosition : vector from the origin of the simulation world
 * - mass : mass of the vertex
 */
struct CustomVertex
{
  int id;
  Vector3 localPosition;
  Vector3 absPosition;
  double mass;
};

/**
 * CUSTOM POLYGON
 *
 * - size : number of vertices forming the polygon
 * - vertices_p : array of pointers to vertices ordered clowise
 */
struct CustomPolygon
{
  int size;
  CustomVertex** vertices_p;

  std::vector<Edge> getEdges() const;
  Plane getPlane() const;
  Vector3 getNormal() const;
};

namespace Geometry
{
  bool areOnTheSameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b);
  bool isInsideTriangle(Vector3 point, Triangle triangle);

  Vector3 closestPointOfEdge(Vector3 point, Edge edge, double* distance_p = NULL);
  Vector3 closestPointOfPlane(Vector3 point, Plane plane, double* distance_p = NULL);
  Vector3 closestPointOfTriangle(Vector3 point, Triangle, double* distance_p = NULL);
  Vector3 closestPointOfPolygon(Vector3 point, Polygon polygon, double* distance = NULL);
  Vector3 closestPointOfTetrahedron(Vector3 point, Tetrahedron tetrahedron, double* distance = NULL);
  Vector3 closestPointOfSphere(Vector3 p, Sphere sphere, double* distance_p = NULL);

  Vector3 supportPoint(std::vector<Vector3> points, Vector3 direction);
  std::vector<Vector3> minkowskiDifference(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p);
  Vector3 gjkDistanceBetweenPolyhedra(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p);
};

#endif

