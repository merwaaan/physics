#include "Simplex.h"

#include "CustomRigidBody.h"

void Simplex::reduce(Vector3 closest)
{
  // the simplex only has one point : it is already in his minimal form
  if(this->points.size() == 1)
    return;

  // the simplex has two points : the edge can de reduced to a single vertex
  if(this->points.size() == 2)
  {
    if(this->points[1] == closest)
	    this->points.erase(this->points.begin());
  }

  // the simplex has three points : the face can de reduced to an edge
  if(this->points.size() == 3)
  {

  }

  // the simplex has four points : the tetrahedron can be reduced to a face
  if(this->points.size() == 4)
  {

  }
}

Vector3 Simplex::closestPointToOrigin() const
{
  // the simplex only has one point : return the point
  if(this->points.size() == 1)
    return this->points[0];
  
  // the simplex has two points : find the closest point on the edge formed by these vertices
  if(this->points.size() == 2)
  {
    Edge e = {this->points[0], this->points[1]};

    return Geometry::closestPointOfEdge(Vector3(0, 0, 0), e);
  }

  // the simplex has three points : find the closest point on the triangle formed by theses vertices
  if(this->points.size() == 3)
  {
	  Triangle triangle;
	  triangle.a = this->points[0];
	  triangle.b = this->points[1];
    triangle.c = this->points[2];

    return Geometry::closestPointOfTriangle(Vector3(0, 0, 0), triangle);
  }
  
  // the simplex has four points : find the closest point on the tetrahedron formed by these vertices
  if(this->points.size() == 4)
  {
    Tetrahedron tetrahedron;
    Triangle a = {this->points[0], this->points[1], this->points[2]};
    Triangle b = {this->points[0], this->points[1], this->points[3]};
    Triangle c = {this->points[0], this->points[2], this->points[3]};
    Triangle d = {this->points[1], this->points[2], this->points[3]};
    tetrahedron.a = a;
    tetrahedron.b = b;
    tetrahedron.c = c;
    tetrahedron.d = d;

    return Geometry::closestPointOfTetrahedron(Vector3(0, 0, 0), tetrahedron);
  }

  return Vector3(0, 0, 0);
}

