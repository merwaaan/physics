#ifndef BOX_H
#define BOX_H

#include "RigidBody.h"

class Box : public RigidBody
{
  private:
    double width;
    double height;
    double depth;

  public:
    Box(double width, double height, double depth);
    ~Box();

    void computeInverseInertiaTensor();
};

class Cube : public Box
{
  public:
    Cube(double side);
    ~Cube();
};

#endif

