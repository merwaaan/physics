#include "Geometry.h"

#include <cmath>

#include "CustomRigidBody.h"

/**
 * Return the closest point of a edge from an arbitrary point p
 */
Vector3 Geometry::closestPointOfEdge(Vector3 p, Edge edge, double* distance_p)
{
  std::cout << "edge test " << p << std::endl;
  Vector3 length = edge.b - edge.a;
  Vector3 normal = length.normalize();
  std::cout << length << normal << std::endl;
  double t = ((p - edge.a) * normal) / length.length();
  t = t < 0 ? 0 : t;
  t = t > 1 ? 1 : t;
  std::cout << t << std::endl;

  if(distance_p != NULL)
    *distance_p = ((edge.a + t * length) - p).length();

  return edge.a + t * length;
}

/**
 * Return the closest point of a plane to an arbitrary point p
 */
Vector3 Geometry::closestPointOfPlane(Vector3 p, Plane plane, double* distance_p)
{
  if(distance_p != NULL)
    *distance_p = -(plane.normal * (plane.point - p)) / plane.normal.length();

  return p - plane.normal * (plane.normal * (p - plane.point));
}

/**
 * Return the closest point of a polygon to an arbitrary point p
 */
Vector3 Geometry::closestPointOfPolygon(Vector3 p, Polygon polygon, double* distance_p)
{
  /*uble d;

  // project the point onto the plane of the polygon
  Plane polygonPlane = polygon.getPlane();
  Vector3 projection = Geometry::closestPointOfPlane(p, polygonPlane, &d);
  std::cout << "projection " << projection << std::endl;

  // if the projected point lies within the polygon, we just found the closest point
  if(Geometry::isPointInsidePolygon(projection, polygon))
  {
    if(distance_p != NULL)
      *distance_p = d;

    return projection;
  }

  std::cout << "point not inside!" << std::endl;

  // else find the closest point of the edges
  std::vector<Edge> edges = polygon.getEdges();

  std::vector<Vector3> points(edges.size());
  std::vector<double> distances(edges.size());

  for(int i = 0; i < edges.size(); ++i)
    points[i] = Geometry::closestPointOfEdge(projection, edges[i], &distances[i]);

  double closestIndex = 0;
  for(int i = 1; i < points.size(); ++i)
    if(distances[i] < distances[closestIndex])
      closestIndex = i;
  
  std::cout << "smallest distance " << distances[closestIndex] << std::endl;
  if(distance_p != NULL)
    *distance_p = distances[closestIndex];

  return points[closestIndex];
*/}

/**
 * Return the closest point of the surface of a sphere to an abitrary point p
 */
Vector3 Geometry::closestPointOfSphere(Vector3 p, Sphere sphere, double* distance_p)
{
  Vector3 distance = p - sphere.getPosition();
  
  return sphere.getPosition() + distance.normalize() * sphere.getRadius();
}

/**
 * Return the vertices of two custom rigid bodies which are within contact distance from the surface of the other body
 */
std::vector<CustomVertex> Geometry::getContactingVertices(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, double tolerance)
{
  /*std::vector<CustomVertex> vertices;

  // find vertices of body #1 within contact distance from body #2
  for(int i = 0; i < rb1_p->structure.vertices.size(); ++i)
    if(Geometry::isContacting(rb1_p->structure.vertices[i], rb2_p, tolerance))
      vertices.push_back(rb1_p->structure.vertices[i]);

  // find vertices of body #2 within contact distance from body #1
  for(int i = 0; i < rb2_p->structure.vertices.size(); ++i)
    if(Geometry::isContacting(rb2_p->structure.vertices[i], rb1_p, tolerance))
      vertices.push_back(rb2_p->structure.vertices[i]);

  return vertices;*/
}

