#ifndef SIMPLEX_H
#define SIMPLEX_H

#include <vector>

#include "Geometry.h"

struct Simplex
{
  std::vector<Vector3> points;

  Vector3 closestPointToOrigin() const;
  void reduce(Vector3 closest);
};

#endif
