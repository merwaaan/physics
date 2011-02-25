#ifndef BOX_H
#define BOX_H

#include "CustomRigidBody.h"

class Box : public CustomRigidBody
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

