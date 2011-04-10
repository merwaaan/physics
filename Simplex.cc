#include "Simplex.h"

#include "CustomRigidBody.h"

/**
 * Returns a support point based on the implicit Minkowki difference of two rigid bodies.
 */
Vector3 Simplex::getSupportPoint(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, Vector3 direction)
{
	return rb1_p->getSupportPoint(direction) - rb2_p->getSupportPoint(-1 * direction);
}

/**
 * Determine the closest point from the origin within the simplex.
 * Reduce the simplex's dimension if possible.
 */
Vector3 Simplex::getClosestPointAndReduce()
{
	Vector3 closest;

	// the simplex has four points : find the closest point on the tetrahedron formed by these vertices
  if(this->points.size() == 4)
	{
		Triangle a = {this->points[1], this->points[2], this->points[3]};
		Triangle b = {this->points[0], this->points[2], this->points[3]};
		Triangle c = {this->points[0], this->points[1], this->points[3]};
		Triangle d = {this->points[0], this->points[1], this->points[2]};
		Tetrahedron tetrahedron = {a, b, c, d};

		closest =  Geometry::closestPointOfTetrahedron(Vector3(0, 0, 0), tetrahedron);

		this->reduceToTriangle(closest, tetrahedron);
	}

	// the simplex has three points : find the closest point on the triangle formed by theses vertices
	if(this->points.size() == 3)
  {
	  Triangle triangle = {this->points[0], this->points[1], this->points[2]};
		closest = Geometry::closestPointOfTriangle(Vector3(0, 0, 0), triangle);

		this->reduceToEdge(closest, triangle);
  }

	// the simplex has two points : find the closest point on the edge formed by these vertices
  if(this->points.size() == 2)
  {
    Edge edge = {this->points[0], this->points[1]};

		closest = Geometry::closestPointOfEdge(Vector3(0, 0, 0), edge);

		this->reduceToPoint(closest);
	}

  // the simplex only has one point : return the point
  if(this->points.size() == 1)
    closest = this->points[0];

	return closest;
}

bool Simplex::reduceToPoint(Vector3 closest)
{
	if(closest == this->points[0])
	{
		std::cout << "reduce to point" << std::endl;
		std::cout << "closest " << closest << std::endl;

		this->points.erase(this->points.begin() + 1);
		return true;
	}

	if(closest == this->points[1])
	{
		this->points.erase(this->points.begin());
		return true;
	}

	return false;
}

bool Simplex::reduceToEdge(Vector3 closest, Triangle triangle)
{
	std::vector<Edge> edges = triangle.getEdges();
	double distance;
	
	for(int i = 0; i < 3; ++i)
	{
		Geometry::closestPointOfEdge(closest, edges[i], &distance);

		if(distance < 0.01)
		{
			this->points.erase(this->points.begin() + (i + 2) % 3);

			return true;
		}
	}

	return false;
}

bool Simplex::reduceToTriangle(Vector3 closest, Tetrahedron tetra)
{
	std::vector<Triangle> triangles = tetra.getTriangles();
	double distance;

	for(int i = 0; i < 4; ++i)
	{
		Geometry::closestPointOfTriangle(closest, triangles[i], &distance);
		if(distance < 0.01)
		{
			std::cout << "reduce to triangle" << std::endl;

			for(int j = 0; j < 4; ++j)
				if(this->points[j] != triangles[i].a && this->points[j] != triangles[i].b && this->points[j] != triangles[i].c)
					this->points.erase(this->points.begin() + j);

			return true;
		}
	}
	
	return false;
}
