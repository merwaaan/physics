#include "Geometry.h"

#include <cmath>

#include "CustomRigidBody.h"
#include "Simplex.h"

Polygon CustomPolygon::getPolygon() const
{
  Polygon polygon;

  for(int i = 0; i < this->size; ++i)
    polygon.points.push_back(this->vertices_p[i]->absPosition);

  return polygon;
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

double Geometry::clamp(double x, double min, double max)
{
  x = x < min ? min : x;
  x = x > max ? max : x;

  return x;
}

/**
 * Return true if the two points are on the same side of an edge
 */
bool Geometry::areOnTheSameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b)
{
  return ((b - a) ^ (p1 - a)) * ((b - a) ^ (p2 - a)) >= 0;
}

/**
 * Return the position of the center of a given set of points
 */
Vector3 Geometry::centerOfPoints(std::vector<Vector3> points)
{
  Vector3 center;

  for(int i = 0; i < points.size(); ++i)
    center += points[i];
  
  center = center / points.size();

  return center;
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
 * Return true if a point lies inside the convex hull given by a set of points
 */
bool Geometry::isInsideConvexHull(Vector3 point, std::vector<Vector3> hull)
{
  Vector3 center = Geometry::centerOfPoints(hull);

  // center the points around the origin
  for(int i = 0; i < hull.size(); ++i)
    hull[i] -= center;
  point -= center;

  for(int i = 0; i < hull.size(); ++i)
  {
	  if(hull[i] * point > hull[i] * hull[i])
    {
      // put back the remaining points to their original positions
      for(int i = 0; i < hull.size(); ++i)
        hull[i] += center;
      point += center;

      return false;
    }
  }

  // put back the remaining points to their original positions
  for(int i = 0; i < hull.size(); ++i)
    hull[i] += center;
  point += center;
  return true;
}

double Geometry::edgeEdgeDistance(Edge edge1, Edge edge2, Vector3* closest1_p, Vector3* closest2_p)
{
  double s, t;

  Vector3 d1 = edge1.b - edge1.a;
  Vector3 d2 = edge2.b - edge2.a;

  Vector3 r = edge1.a - edge2.a;
  double a = d1 * d1;
  double e = d2 * d2;
  double f = d2 * r;

  double tolerance = 0.001;

  if(a <= tolerance && e <= tolerance)
  {
    s = t = 0;
    *closest1_p = edge1.a;
    *closest2_p = edge2.a;

    return (*closest1_p - *closest2_p) * (*closest1_p - *closest2_p);
  }

  if(a <= tolerance)
  {
    s = 0;
    t = Geometry::clamp(f / e, 0, 1);
  }
  else
  {
    double c = d1 * r;
    
    if(e <= tolerance)
    {
      t = 0;
      s = Geometry::clamp(-c / a, 0, 1);
    }
    else
    {
      double b = d1 * d2;
      double denominator = a * e - b * b;

      if(denominator != 0)
        s = Geometry::clamp((b * f - c * e) / denominator, 0, 1);
      else
        s = 0;

      t = (b * s + f) / e;

      if(t < 0)
      {
        t = 0;
        s = Geometry::clamp(-c / a, 0, 1);
      }
      else if(t > 1)
      {
        t = 1;
        s = Geometry::clamp((b - c) / a, 0, 1);
      }
    }
  }

  *closest1_p = edge1.a + d1 * s;
  *closest2_p = edge2.a + d2 * t;

  return (*closest1_p - *closest2_p) * (*closest1_p - *closest2_p);
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
 * Return the closest point of a plane to a point p
 */
Vector3 Geometry::closestPointOfPlane(Vector3 point, Plane plane, double* distance_p)
{
  double t = plane.normal * (point - plane.point);
  Vector3 closest = point - (t * plane.normal);

  if(distance_p != NULL)
    *distance_p = t;

  return closest;
}

/**
 * Return the closest point of a triangle to a point p
 */
Vector3 Geometry::closestPointOfTriangle(Vector3 point, Triangle triangle, double* distance_p)
{
  // project the point onto the plane extending the triangle
  Plane plane = {triangle.a, ((triangle.b - triangle.a) ^ (triangle.c - triangle.a)).normalize()};
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

  Vector3 closest = pab;
  double closestDistance = dab;
  if(dbc < closestDistance)
  {
    closest = pbc;
    closestDistance = dbc;
  }
  if(dca < closestDistance)
    closest = pca;

  return closest;
}

/**
 * Return the closest point of a polygon to a point p
 */
Vector3 Geometry::closestPointOfPolygon(Vector3 point, Polygon polygon, double* distance_p)
{
  // find the center of the polygon
  Vector3 center;
  for(int i = 0; i < polygon.points.size(); ++i)
    center += polygon.points[i];
  center = center / polygon.points.size();

  // subvidide the polygon into an triangle fan
  std::vector<Triangle> triangles;
  for(int i = 0; i < polygon.points.size() - 1; ++i)
    triangles.push_back((Triangle){polygon.points[i], center, polygon.points[i + 1]});
  triangles.push_back((Triangle){polygon.points[polygon.points.size() - 1], center, polygon.points[0]});

  // find the closest point for each triangle
  std::vector<Vector3> closests(triangles.size());
  std::vector<double> distances(triangles.size());
  for(int i = 0; i < triangles.size(); ++i)
  {
    closests[i] = Geometry::closestPointOfTriangle(point, triangles[i], &distances[i]);
    std::cout << distances[i] << std::endl;
  }
  
  // only keep the closest point
  Vector3 closest = closests[0];
  double bestDistance = distances[0];
  for(int i = 1; i < triangles.size(); ++i)
    if(distances[i] < bestDistance)
    {
      bestDistance = distances[i];
      closest = closests[i];
    }

  if(distance_p != NULL)
    *distance_p = bestDistance;

  return closest;
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
  Vector3 center = Geometry::centerOfPoints(points);

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

Vector3 Geometry::gjkDistanceBetweenPolyhedra(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, bool* interPenetration_p)
{
  // compute the convex hull of the Minkowski difference
  std::vector<Vector3> minkowski = Geometry::minkowskiDifference(rb1_p, rb2_p);

  // initialize the simplex to a random point and start the search from it
  Simplex simplex; 
  simplex.points.push_back(minkowski[0]);

  Vector3 closest = simplex.points[0];

  while(true)
  {
    // find the closest point to the origin and a support point along its direction to the origin
    Vector3 directionToOrigin = Vector3(0, 0, 0) - closest;
    Vector3 support = Geometry::supportPoint(minkowski, directionToOrigin);

    // terminate when the support point is already contained within the simplex
    for(int i = 0; i < simplex.points.size(); ++i)
      if(simplex.points[i] == support)
        return directionToOrigin;

    // add the support point to the simplex
    simplex.points.push_back(support);

    closest = simplex.closestPointToOrigin();

    // reduce the simplex to a minimum simplex by getting rid of the vertices which
    // are not part of the definition of the new support point
    simplex.reduce(closest);
  }
}

std::vector<Contact> Geometry::vertexFaceContacts(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, double tolerance, bool second)
{
  std::vector<Contact> contacts;
  std::cout << rb2_p->structure.polygons.size() << " poly" << std::endl;
  std::cout << rb1_p->structure.vertices.size() << " vert" << std::endl;

  for(int i = 0; i < rb2_p->structure.polygons.size(); ++i)
    for(int j = 0; j < rb1_p->structure.vertices.size(); ++j)
    {
      Vector3 vertex = rb1_p->structure.vertices[j].absPosition;
      Polygon face = rb2_p->structure.polygons[i].getPolygon();

      double distance;
      Vector3 point = Geometry::closestPointOfPolygon(vertex, face, &distance);
      //std::cout << distance << std::endl;
      if(distance < tolerance)
      {
        Contact contact;
        contact.a = rb1_p;
        contact.b = rb2_p;
        contact.position = (point + vertex) / 2;
        contact.normal = (point - vertex).normalize();

        contacts.push_back(contact);  
      }
    }

  // if it's the first passage, recursively call the method to obtain the contacts from
  // the perspective of the other body and append them to the list
  if(!second)
  {
    std::vector<Contact> contacts2 = Geometry::vertexFaceContacts(rb2_p, rb1_p, tolerance, true);

    for(int i = 0; i < contacts2.size(); ++i)
      contacts.push_back(contacts2[i]);
  }
 
  return contacts;
}

std::vector<Contact> Geometry::edgeEdgeContacts(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, double tolerance)
{
  std::vector<Contact> contacts;

  std::vector<Edge> edges1 = rb1_p->structure.getEdges();
  std::vector<Edge> edges2 = rb2_p->structure.getEdges();

  for(int i = 0; i < edges1.size(); ++i)
	  for(int j = 0; j < edges2.size(); ++j)
    {
	    Vector3 closest1, closest2;
	    double distance = Geometry::edgeEdgeDistance(edges1[i], edges2[j], &closest1, &closest2);

	    if(distance < tolerance)
	    {
		    Contact contact;
		    contact.a = rb1_p;
		    contact.b = rb2_p;
		    contact.position = (closest1 + closest2) / 2;

		    Vector3 v1 = edges1[i].b - edges1[i].a;
		    Vector3 v2 = edges2[j].b - edges2[j].a;
		    contact.normal = v1 ^ v2;

		    contacts.push_back(contact);
	    }
    }

  return contacts;
}
