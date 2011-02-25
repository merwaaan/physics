#ifndef CUBE_H
#define CUBE_H

#include "RigidBody.h"

class Cube : public RigidBody
{
  private:
    double side;
    double h; // side / 2

  public:
    Cube(double side);
    ~Cube();

    void computeInverseInertiaTensor();
};

#endif

