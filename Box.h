#ifndef BOX_H
#define BOX_H

#include "CustomRigidBody.h"

class Box : public CustomRigidBody
{
  public:
	  double width;
    double height;
    double depth;

  public:
    Box(double width, double height, double depth, double mass);
    ~Box();

    void computeInverseInertiaTensor();
};

class Cube : public Box
{
  public:
	Cube(double side, double mass);
    ~Cube();
};

#endif

