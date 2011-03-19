#include "Geometry.h"

#include <cmath>

#include "CustomRigidBody.h"
#include "Simplex.h"

/**
 * Return true if the two points are on the same side of an edge
 */
bool Geometry::areOnTheSameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b)
{
  return ((b - a) ^ (p1 - a)) * ((b - a) ^ (p2 - a)) >= 0;
}

/**
 * Return true if a point lies inside a triangle
 */
bool Geometry::isInsideTriangle(Vector3 point, Triangle triangle)
{
  // check if the point lies inside the positive half-spaces of each edges
  return
    Geometry::areOnTheSameSide(point, triangle.a, triangle.b, triangle.c) &&
    Geometry::areOnTheSameSide(point, triangle.b, triangle.a, triangle.c) &&
    Geometry::areOnTheSameSide(point, triangle.c, triangle.b, triangle.a);
}

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
 * Return the normal of the polygon (outside-oriented)
 */
Vector3 CustomPolygon::getNormal() const
{
  Vector3 v1 = this->vertices_p[0]->absPosition - this->vertices_p[1]->absPosition;
  Vector3 v2 = this->vertices_p[1]->absPosition - this->vertices_p[2]->absPosition;

  return (v1 ^ v2).normalize();
}

/**
 * Return the closest point of an edge from a point point p
 */
Vector3 Geometry::closestPointOfEdge(Vector3 point, Edge edge, double* distance_p)
{
  Vector3 length = edge.b - edge.a;
  Vector3 normal = length.normalize();

  double t = ((point - edge.a) * normal) / length.length();
  t = t < 0 ? 0 : t;
  t = t > 1 ? 1 : t;

  if(distance_p != NULL)
    *distance_p = ((edge.a + t * length) - point).length();

  return edge.a + t * length;
}

/**
 * Return the closest point of a plane to a point point p
 */
Vector3 Geometry::closestPointOfPlane(Vector3 point, Plane plane, double* distance_p)
{
  Vector3 closest = point - plane.normal * (plane.normal * (point - plane.point));

  if(distance_p != NULL)
    *distance_p = (closest - point).length();
    //*distance_p = -(plane.normal * (plane.point - p)) / plane.normal.length();

  return closest;
}

/**
 * Return the closest point of a triangle to a point p
 */
Vector3 Geometry::closestPointOfTriangle(Vector3 point, Triangle triangle, double* distance_p)
{
  // project the point onto the plane extending the triangle
  Plane plane = {triangle.a, ((triangle.a - triangle.b) ^ (triangle.b - triangle.c)).normalize()};
  Vector3 projection = Geometry::closestPointOfPlane(point, plane);

  // if the projection lies within the triangle border, the closest point is the projection
  if(Geometry::isInsideTriangle(projection, triangle))
    return projection;

  // else the closest point lies on one of the edges
  Edge ab = {triangle.a, triangle.b};
  Edge bc = {triangle.b, triangle.c};
  Edge ca = {triangle.c, triangle.a};
  double dab, dbc, dca;

  Vector3 pab = Geometry::closestPointOfEdge(point, ab, &dab);
  Vector3 pbc = Geometry::closestPointOfEdge(point, bc, &dbc);
  Vector3 pca = Geometry::closestPointOfEdge(point, ca, &dca);

  Vector3 closest;
  double closestDistance;
  if(dab < dbc)
  {
    closest = pab;
    closestDistance = dab;
  }
 
  return dca < closestDistance ? pca : closest;
}

/**
 * Return the closest point of a polygon to a point p
 */
Vector3 Geometry::closestPointOfPolygon(Vector3 point, Polygon polygon, double* distance_p)
{
  
}

/**
 * Return the closest point of a tetrahedron to a point p
 */
Vector3 Geometry::closestPointOfTetrahedron(Vector3 point, Tetrahedron tetra, double* distance_p)
{
  
}

/**
 * Return the closest point of a sphere to a point p
 */
Vector3 Geometry::closestPointOfSphere(Vector3 point, Sphere sphere, double* distance_p)
{
  Vector3 centerDistance = point - sphere.getPosition();

  // if the point is inside the sphere, it is already the closest point
  if(centerDistance.length() < sphere.getRadius())
  {
    if(distance_p != NULL)
      *distance_p = centerDistance.length();

    return point;
  }

  // else we hav to find the surface point the closest to the point
  Vector3 closest = sphere.getPosition() + centerDistance.normalize() * sphere.getRadius();

  if(distance_p != NULL)
    *distance_p = (closest - sphere.getPosition()).length();

  return closest;
}

/**
 * Return a support point by choosing the farthest in a specified direction
 */
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
 * Return a set of points describing the convex hull of the original set of points
 */
std::vector<Vector3> Geometry::convexHull(std::vector<Vector3> points)
{
  // compute the center of the set of points
  Vector3 center;
  for(int i = 0; i < points.size(); ++i)
    center += points[i];
  center = center / points.size();

  // center the points around the origin
  for(int i = 0; i < points.size(); ++i)
    points[i] -= center;

  // compute the convex hull by only keeping extrem points
  for(int i = 0; i < points.size(); ++i)
  {
    for(int j = 0; j < points.size(); ++j)
    {
	    if(i != j && points[i] * points[i] <= points[i] * points[j])
      {
	      points.erase(points.begin() + i--);
        break;
      }
    }
  }

  // put back the remaining points to their original positions
  for(int i = 0; i < points.size(); ++i)
    points[i] += center;

  return points;
}

/**
 * Return a set of points describing the Minkowski difference between two bodies
 */
std::vector<Vector3> Geometry::minkowskiDifference(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p)
{
  // compute all the points of the Minkowski difference
  std::vector<Vector3> points;
  for(int i = 0; i < rb1_p->structure.vertices.size(); ++i)
    for(int j = 0; j < rb2_p->structure.vertices.size(); ++j)
      points.push_back(rb2_p->structure.vertices[j].absPosition - rb1_p->structure.vertices[i].absPosition);

  return Geometry::convexHull(points);
}

Vector3 Geometry::gjkDistanceBetweenPolyhedra(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p)
{
  // compute the convex hull of the Minkowski difference
  std::vector<Vector3> minkowski = Geometry::minkowskiDifference(rb1_p, rb2_p);

  // initialize the simplex to a random point
  Simplex simplex; 
  simplex.points.push_back(minkowski[0]);

  while(true)
  {
    // find the closest point to the origin and a support point along its direction to the origin
    Vector3 closest = simplex.closestPointToOrigin();
    Vector3 directionToOrigin = Vector3(0, 0, 0) - closest;
    Vector3 support = Geometry::supportPoint(minkowski, directionToOrigin);

    // terminate when the support point is already contained within the simplex
    for(int i = 0; i < simplex.points.size(); ++i)
      if(simplex.points[i] == support)
        return directionToOrigin;

    // add the support point to the simplex
    simplex.points.push_back(support);

    // reduce the simplex to a minimum simplex by getting rid of the vertices which
    // are not part of the definition of the new support point
    simplex.reduce(closest);
  }
}

