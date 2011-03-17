#include "Geometry.h"

#include <cmath>

#include "CustomRigidBody.h"
#include "Simplex.h"

/**
 * Return a list of the edges forming the polygon perimeter
 */
std::vector<Edge> CustomPolygon::getEdges() const
{
  std::vector<Edge> edges;

  for(int i = 0; i < size - 1; ++i)
  {
    Edge s = {this->vertices_p[i]->absPosition, this->vertices_p[i + 1]->absPosition};
    edges.push_back(s);
  }

  Edge s = {this->vertices_p[size - 1]->absPosition, this->vertices_p[0]->absPosition};
  edges.push_back(s);

  return edges;
}

/**
  * Return the plane extending the polygon
  */
Plane CustomPolygon::getPlane() const
{
  Plane sp;

  sp.point = this->vertices_p[0]->absPosition;
  sp.normal = this->getNormal();

  return sp;
}

/**
 * Return the normal of the polygon
 */
Vector3 CustomPolygon::getNormal() const
{
  Vector3 v1 = this->vertices_p[0]->absPosition - this->vertices_p[1]->absPosition;
  Vector3 v2 = this->vertices_p[1]->absPosition - this->vertices_p[2]->absPosition;

  return (v1 ^ v2).normalize();
}

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

Vector3 Geometry::supportPoint(std::vector<Vector3> points, Vector3 direction)
{
  int bestIndex = 0;
  double bestLength = points[0] * direction;

  for(int i = 1; i < points.size(); ++i)
  {
    double length = points[i] * direction;

    if(length > bestLength)
    {
      bestIndex = i;
      bestLength = length;
    }
  }

  return points[bestIndex];
}

/**
 * Return a set of points describing the convex hull of the Minkowski difference
 * between two bodies
 */
std::vector<Vector3> minkowskiDifference(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p)
{
  // compute all the points of the Minkowski difference
  std::vector<Vector3> points;
  for(int i = 0; i < rb1_p->structure.vertices.size(); ++i)
    for(int j = 0; j < rb2_p->structure.vertices.size(); ++j)
      points.push_back(rb2_p->structure.vertices[j].absPosition - rb1_p->structure.vertices[i].absPosition);

  // compute the center of the set of points
  Vector3 center;
  for(int i = 0; i < points.size(); ++i)
    center += points[i];
  center = center / points.size();
  
  // compute the convex hull by only keeping extrem points
  std::vector<int> pointsToBeRemoved;
  for(int i = 0; i < points.size(); ++i)
  {
    Vector3 direction = center - points[i];

    for(int j = 0; j < points.size(); ++j)
      if(i != j && (points[i] * direction).length() < (points[j] * direction).length())
      {
        pointsToBeRemoved.push_back(i);
        break;
      } 
  }

  for(int i = 0; i < pointsToBeRemoved.size(); ++i)
    points.erase(points.begin() + pointsToBeRemoved[i]);

  return points;
}

Vector3 Geometry::gjkDistanceBetweenPolyhedra(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p)
{
  // compute the convex hull of the Minkowski difference
  std::vector<Vector3> minkowski = Geometry::minkowskiDifference(rb1_p, rb2_p);

  // initialize the simplex to a random point
  Simplex simplex; 
  simplex.points.push_back(minkowski[0]);

  // find the closest point to the origin
  Vector3 closest = simplex.closestPointToOrigin();

  // find a support point (most extrem point along the direction to the origin)
  Vector3 directionToOrigin = Vector3(0, 0, 0) - closest;
  simplex.addSupportPoint(Geometry::supportPoint(minkowski, directionToOrigin));
}

