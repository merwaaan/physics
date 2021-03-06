#include "Simplex.h"

#include "CustomRigidBody.h"

/**
 * Determine the closest point from the origin within the simplex.
 */
Vector3 Simplex::getClosestPoint()
{
	// The simplex has four points : find the closest point on the tetrahedron formed by these vertices.
  if(this->points.size() == 4)
	{
		Triangle a = {this->points[1], this->points[2], this->points[3]};
		Triangle b = {this->points[0], this->points[2], this->points[3]};
		Triangle c = {this->points[0], this->points[1], this->points[3]};
		Triangle d = {this->points[0], this->points[1], this->points[2]};
		Tetrahedron tetrahedron = {a, b, c, d};

		return Geometry::closestPointOfTetrahedron(Vector3(0, 0, 0), tetrahedron);
	}

	// The simplex has three points : find the closest point on the triangle formed by theses vertices.
	if(this->points.size() == 3)
  {
	  Triangle triangle = {this->points[0], this->points[1], this->points[2]};

		return Geometry::closestPointOfTriangle(Vector3(0, 0, 0), triangle);
  }

	// The simplex has two points : find the closest point on the edge formed by these vertices.
  if(this->points.size() == 2)
  {
    Edge edge = {this->points[0], this->points[1]};

		return Geometry::closestPointOfEdge(Vector3(0, 0, 0), edge);
	}

  // The simplex only has one point : return the point.
  if(this->points.size() == 1)
		return this->points[0];
}

void Simplex::reduce(Vector3 closest)
{
  if(this->points.size() == 4)
	{
		Triangle a = {this->points[1], this->points[2], this->points[3]};
		Triangle b = {this->points[0], this->points[2], this->points[3]};
		Triangle c = {this->points[0], this->points[1], this->points[3]};
		Triangle d = {this->points[0], this->points[1], this->points[2]};
		Tetrahedron tetrahedron = {a, b, c, d};

		this->reduceToTriangle(closest, tetrahedron);
	}

	if(this->points.size() == 3)
	{
	  Triangle triangle = {this->points[0], this->points[1], this->points[2]};

		this->reduceToEdge(closest, triangle);
	}

  if(this->points.size() == 2)
		this->reduceToPoint(closest);
}

bool Simplex::reduceToPoint(Vector3 closest)
{
	if(closest == this->points[0])
	{
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
			for(int j = 0; j < 4; ++j)
				if(this->points[j] != triangles[i].a && this->points[j] != triangles[i].b && this->points[j] != triangles[i].c)
					this->points.erase(this->points.begin() + j);

			return true;
		}
	}
	
	return false;
}
