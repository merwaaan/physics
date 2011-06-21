#ifndef CYLINDER_H
#define CYLINDER_H

#include "CustomRigidBody.h"

class Cylinder : public CustomRigidBody
{
  protected:
    double radius;
	  int sides;
		double height;

  public:
    Cylinder(double radius, int sides, double height, double mass);
    ~Cylinder();

    void computeInverseInertiaTensor();

		double getRadius() { return this->radius; }
		double getSides() { return this->sides; }
		double getHeight() { return this->height; }
};

#endif
