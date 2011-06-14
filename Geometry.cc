#include "Geometry.h"

#include <cmath>

#include "CustomRigidBody.h"
#include "Engine.h"
#include "Simplex.h"

extern Engine* E;

/**
 * Return an integer representing the half-side of the plane on which
 * the body is lying.
 *
 * -1 : negative half-space
 * +1 : positive half-space
 *  0 : the body lies in the two half-spaces
 *
 * (from The Method of Separating Axis - David Eberly)
 */
int Plane::whichHalfSpace(CustomRigidBody* rb_p)
{
	int pos = 0;
	int neg = 0;
	double distance;

	for(int i = 0; i < rb_p->structure.vertices.size(); ++i)
	{
		Geometry::closestPointOfPlane(rb_p->structure.vertices[i].absPosition, *this, &distance);

		if(distance < 0)
			++neg;
		else
			++pos;

		if(pos && neg)
			return 0;
	}

	return pos ? 1 : -1;
}

Vector3 Triangle::getCenter() const
{
	return (this->a + this->b + this->c) / 3;
}

Plane Triangle::getPlane() const
{
  Plane p;

  p.point = this->a;
  p.normal = ((this->a - this->b) ^ (this->b - this->c)).normalize();

  return p;
}

std::vector<Edge> Triangle::getEdges() const
{
	std::vector<Edge> edges(3);

	edges[0] = (Edge){this->a, this->b};
	edges[1] = (Edge){this->b, this->c};
	edges[2] = (Edge){this->c, this->a};

	return edges;
}

std::vector<Triangle> Tetrahedron::getTriangles() const
{
	std::vector<Triangle> triangles(4);

	triangles[0] = this->a;
	triangles[1] = this->b;
	triangles[2] = this->c;
	triangles[3] = this->d;

	// restructure the triangles so that their normals are always directed outward
	Vector3 center;
	for(int i = 0; i < 4; ++i)
		center += triangles[i].getCenter();
	center *= 0.25;

	for(int i = 0; i < 4; ++i)
		if(triangles[i].getPlane().normal * (center - triangles[i].a) > 0)
		{
			Vector3 t = triangles[i].b;
			triangles[i].b = triangles[i].c;
			triangles[i].c = t;
		}

	return triangles;
}

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
  Plane p;

  p.point = this->vertices_p[0]->absPosition;
  p.normal = this->getNormal();

  return p;
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

bool Geometry::isInsideTetrahedron(Vector3 point, Tetrahedron tetra)
{
	std::vector<Triangle> triangles = tetra.getTriangles();

	std::vector<Plane> planes(4);
	for(int i = 0; i < 4; ++i)
		planes[i] = triangles[i].getPlane();

	double distance;
	for(int i = 0; i < 4; ++i)
	{
		Geometry::closestPointOfPlane(point, planes[i], &distance);

		if(distance > 0)
			return false;
	}

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

  if(a <= E->getTolerance() && e <= E->getTolerance())
  {
    s = t = 0;
    *closest1_p = edge1.a;
    *closest2_p = edge2.a;

    return (*closest1_p - *closest2_p) * (*closest1_p - *closest2_p);
  }

  if(a <= E->getTolerance())
  {
    s = 0;
    t = Geometry::clamp(f / e, 0, 1);
  }
  else
  {
    double c = d1 * r;
    
    if(e <= E->getTolerance())
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
  Vector3 closest = point - t * plane.normal;

  if(distance_p != NULL)
    *distance_p = t;

  return closest;
}

/**
 * Return the closest point of a triangle to a point p
 *
 * From Real-Time Collision Detection - Christer Ericson
 */
Vector3 Geometry::closestPointOfTriangle(Vector3 point, Triangle triangle, double* distance_p)
{
	Vector3 originalPoint = point;

	// project the point onto the plane extending the triangle
	point = Geometry::closestPointOfPlane(originalPoint, triangle.getPlane());

	Vector3 ab = triangle.b - triangle.a;
	Vector3 ac = triangle.c - triangle.a;
	Vector3 ap = point - triangle.a;

	// is the point in A's voronoï region?
	double d1 = ab * ap;
	double d2 = ac * ap;
 	if(d1 <= 0 && d2 <= 0)
	{
		if(distance_p != NULL)
			*distance_p = (originalPoint - triangle.a).length();

		return triangle.a;
	}
	
	// is the point in B's voronoï region?
	Vector3 bp = point - triangle.b;
	double d3 = ab * bp;
	double d4 = ac * bp;
	if(d3 >= 0 && d4 <= d3)
	{
		if(distance_p != NULL)
			*distance_p = (originalPoint - triangle.b).length();

		return triangle.b;
	}

	// is the point in C's voronoï region?
	Vector3 cp = point - triangle.c;
	double d5 = ab * cp;
	double d6 = ac * cp;
	if(d6 >= 0 && d5 <= d6)
	{
		if(distance_p != NULL)
			*distance_p = (originalPoint - triangle.c).length();

		return triangle.c;
	}

	// is the point in AB's voronoï region?
	double vc = d1 * d4 - d3 * d2;
	if(vc <= 0 && d1 >= 0 && d3 <= 0)
	{
		double v = d1 / (d1 - d3);
		Vector3 closest = triangle.a + v * ab;

		if(distance_p != NULL)
			*distance_p = (originalPoint - closest).length();

		return closest;
	}

	// is the point in AC's voronoï region?
	double vb = d5 * d2 - d1 * d6;
	if(vb <= 0 && d2 >= 0 && d6 <= 0)
	{
		double w = d2 / (d2 - d6);
		Vector3 closest = triangle.a + w * ac;

		if(distance_p != NULL)
			*distance_p = (originalPoint - closest).length();

		return closest;
	}

	// is the point in BC's voronoï region?
	double va = d3 * d6 - d5 * d4;
	if(va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
	{
		double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		Vector3 closest = triangle.b + w * (triangle.c - triangle.b);

		if(distance_p != NULL)
			*distance_p = (originalPoint - closest).length();
		
		return closest;
	}
	
	// the point is inside of the triangle
	double denom = 1.0 / (va + vb + vc);
	double v = vb * denom;
	double w = vc * denom;
	Vector3 closest = triangle.a + ab * v + ac * w;

	if(distance_p != NULL)
		*distance_p = (originalPoint - closest).length();

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

  // subvidide the polygon into a triangle fan
  std::vector<Triangle> triangles;
  for(int i = 0; i < polygon.points.size() - 1; ++i)
    triangles.push_back((Triangle){polygon.points[i], center, polygon.points[i + 1]});
  triangles.push_back((Triangle){polygon.points[polygon.points.size() - 1], center, polygon.points[0]});

  // find the closest point of each triangle
  std::vector<Vector3> closests(triangles.size());
  std::vector<double> distances(triangles.size());
  for(int i = 0; i < triangles.size(); ++i)
    closests[i] = Geometry::closestPointOfTriangle(point, triangles[i], &distances[i]);
  
  // only keep the closest point
	int indexClosest = 0;
  for(int i = 1; i < triangles.size(); ++i)
    if(distances[i] < distances[indexClosest])
			indexClosest = i;

  if(distance_p != NULL)
    *distance_p = distances[indexClosest];

  return closests[indexClosest];
}

/**
 * Return the closest point of a tetrahedron to a point p
 */
Vector3 Geometry::closestPointOfTetrahedron(Vector3 point, Tetrahedron tetra, double* distance_p)
{
	// return the point if it is lying within the tetrahedron borders
	if(Geometry::isInsideTetrahedron(point, tetra))
	{
		if(distance_p != NULL)
			*distance_p = 0;

		return point;
	}
	
	// compute the distances from the point to each triangles
	std::vector<Triangle> triangles = tetra.getTriangles();
	Vector3 closests[4];
	double distances[4];
	for(int i = 0; i < 4; ++i)
		closests[i] = Geometry::closestPointOfTriangle(point, triangles[i], &distances[i]);

	// only keep the closest one
	int indexClosest = 0;
	for(int i = 1; i < 4; ++i)
		if(distances[i] < distances[indexClosest])
			indexClosest = i;

	if(distance_p != NULL)
		*distance_p = distances[indexClosest];

	return closests[indexClosest];
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

Vector3 Geometry::gjkDistance(RigidBody* rb1_p, RigidBody* rb2_p, bool* interPenetration_p)
{
	Vector3 origin(0, 0, 0);

	// Initialize the simplex to a random point from the Minkowski difference.
	Simplex simplex;
	Vector3 startDir(0, -1, 0);
	Vector3 startVertex = rb1_p->getSupportPoint(startDir) - rb2_p->getSupportPoint(-1 * startDir);
	simplex.points.push_back(startVertex);

	while(true)
	{
		// Compute the closest point.
		Vector3 closest = simplex.getClosestPoint();

		// Return an intersection if the closest point is the origin.
		if(closest == origin)
		{
			if(interPenetration_p != NULL)
				*interPenetration_p = true;

			return origin;
		}

		// Reduce the simplex to its minimum form.
		simplex.reduce(closest);

		// Compute a support point along the direction from the closest point to the origin.
		Vector3 support = rb1_p->getSupportPoint(-1 * closest) - rb2_p->getSupportPoint(closest);

		// Return a non-intersection if
		if((-1 * closest) * closest + 0.01 >= (-1 * closest) * support)
		{
			if(interPenetration_p != NULL)
				*interPenetration_p = false;
b
			return closest;
		}

		// Add the closest point to the simplex.
		simplex.points.push_back(support);
	}
}

std::vector<Contact> Geometry::vertexFaceContacts(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, bool second)
{
  std::vector<Contact> contacts;
  for(int i = 0; i < rb2_p->structure.polygons.size(); ++i)
    for(int j = 0; j < rb1_p->structure.vertices.size(); ++j)
    {
      Polygon face = rb2_p->structure.polygons[i].getPolygon();
      Vector3 vertex = rb1_p->structure.vertices[j].absPosition;

      double distance;
      Vector3 point = Geometry::closestPointOfPolygon(vertex, face, &distance);

      if(distance < 0.1)
      {
        Contact contact;

				if(second)
				{
					contact.a = rb1_p;
					contact.b = rb2_p;
				}
				else
				{
					contact.a = rb2_p;
					contact.b = rb1_p;
				}

        contact.position = point;
        contact.normal = (vertex - point).normalize();

        contacts.push_back(contact);  
      }
    }

  // if it's the first passage, recursively call the method to obtain contacts from
  // the perspective of the other body and append them to the list
  if(!second)
  {
    std::vector<Contact> contacts2 = Geometry::vertexFaceContacts(rb2_p, rb1_p, true);

    for(int i = 0; i < contacts2.size(); ++i)
      contacts.push_back(contacts2[i]);
  }
 
  return contacts;
}

std::vector<Contact> Geometry::edgeEdgeContacts(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p)
{
  std::vector<Contact> contacts;

  std::vector<Edge> edges1 = rb1_p->getEdges();
  std::vector<Edge> edges2 = rb2_p->getEdges();

  for(int i = 0; i < edges1.size(); ++i)
	  for(int j = 0; j < edges2.size(); ++j)
    {
	    Vector3 closest1, closest2;
	    double distance = Geometry::edgeEdgeDistance(edges1[i], edges2[j], &closest1, &closest2);

			if(
				edges1[i].a == closest1 ||
				edges1[i].b == closest1 ||
				edges2[j].a == closest2 ||
				edges2[j].b == closest2)
				continue;

			if(distance < 0.1)
			{
		    Contact contact;

		    contact.a = rb1_p;
		    contact.b = rb2_p;
		    contact.position = closest1;

		    Vector3 e1 = edges1[i].b - edges1[i].a;
		    Vector3 e2 = edges2[j].b - edges2[j].a;
		    contact.normal = (e1 ^ e2).normalize();

				// Make sure the contact normal is directed toward the first body
		    Vector3 directionToFirst = contact.a->getPosition() - contact.position;
				if(contact.normal * directionToFirst > contact.normal.negate() * directionToFirst)
					contact.normal = contact.normal.negate();

		    contacts.push_back(contact);
	    }
    }

  return contacts;
}
