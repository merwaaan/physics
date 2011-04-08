#ifndef SIMPLEX_H
#define SIMPLEX_H

#include <vector>

#include "Geometry.h"

struct Simplex
{
  std::vector<Vector3> points;

	Vector3 getSupportPoint(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p, Vector3 direction);
  Vector3 getClosestPointAndReduce();

	bool reduceToPoint(Vector3 closest);
	bool reduceToEdge(Vector3 closest, Triangle triangle);
	bool reduceToTriangle(Vector3 closest, Tetrahedron tetra);
};

#endif
