#ifndef BOX_H
#define BOX_H

#include "CustomRigidBody.h"

class Box : public CustomRigidBody
{
  protected:
	  double width;
    double height;
    double depth;

  public:
    Box(double width, double height, double depth, double mass);
    ~Box();
		virtual Box* copy();

    void computeInverseInertiaTensor();

		double getWidth() { return this->width; }
		double getHeight() { return this->height; }
		double getDepth() { return this->depth; }
};

class Cube : public Box
{
public:
	Cube(double side, double mass);
	~Cube();
};

#endif

