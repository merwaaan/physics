#include "Simplex.h"

#include <cassert>

#include "CustomRigidBody.h"

Simplex::Simplex()
{
}

Simplex::~Simplex()
{
}

void Simplex::addSupportPoint(CustomRigidBody* rb_p, Vector3 point)
{
  Vector3 direction = Vector3(0, 0, 0) - point; 

  Vector3 support;
  Vector3 distance;

  for(int i = 0; i < rb_p->structure.vertices.size(); ++i)
  {
    Vector3 projection = (rb_p->structure.vertices[i].absPosition, direction);

    if(projection.length() > distance.length())
    {
      distance = projection;
      support = projection;
    }
  }

  this->reduce(support);

  this->points.push_back(support);
}

void Simplex::reduce(Vector3 point)
{

}

Vector3 Simplex::closestPointFromOrigin() const
{
  // the simplex only has one point : return the point
  if(this->points.size() == 1)
    return this->getPoint();
  // the simplex has two points : find the closest point on the edge formed by these vertices
  else if(this->points.size() == 2)
    return Geometry::closestPointOfEdge(Vector3(0, 0, 0), this->getEdge());
  // the simplex has three points : find the closest point on the polygon formed by theses vertices
  else if(this->points.size() == 3)
  {
    Polygon polygon;
    polygon.size = 3; 

    return Geometry::closestPointOfPolygon(Vector3(0, 0, 0), polygon);
  }
  // the simplex has four points : find the closest point on the tetrahedron formed by these vertices
  else if(this->points.size() == 4)
  {

  }

  return Vector3(0, 0, 0);
}

Vector3 Simplex::getPoint() const
{
  assert(this->points.size() == 1);

  return this->points[0];
}

Edge Simplex::getEdge() const
{
  Edge edge = {this->points[0], this->points[1]};

  return edge;
}

Polygon Simplex::getPolygon() const
{

}

