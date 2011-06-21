#include "Box.h"

Box::Box(double width, double height, double depth, double mass) :
  width(width),
  height(height),
  depth(depth)
{
	double hw = width / 2;
	double hh = height / 2;
	double hd = depth / 2;
	double m = mass / 8;

  this->addVertex(0, -hw, hh, hd, m);
  this->addVertex(1, -hw, -hh, hd, m);
  this->addVertex(2, hw, -hh, hd, m);
  this->addVertex(3, hw, hh, hd, m);
  this->addVertex(4, -hw, hh, -hd, m);
  this->addVertex(5, -hw, -hh, -hd, m);
  this->addVertex(6, hw, -hh, -hd, m);
  this->addVertex(7, hw, hh, -hd, m);
  
  this->addPolygon(4, (int[]){0, 1, 2, 3}); // front
  this->addPolygon(4, (int[]){7, 6, 5, 4}); // back
  this->addPolygon(4, (int[]){4, 0, 3, 7}); // top
  this->addPolygon(4, (int[]){1, 5, 6, 2}); // bottom
  this->addPolygon(4, (int[]){4, 5, 1, 0}); // left
  this->addPolygon(4, (int[]){7, 3, 2, 6}); // right
}

Box::~Box()
{
}

void Box::computeInverseInertiaTensor()
{
  Matrix3 inertiaTensor;

	double m = (1/this->inverseMass) / 12;
	double sh = this->height * this->height;
	double sw = this->width * this->width;
	double sd = this->depth * this->depth;

  inertiaTensor.set(0, 0, m * (sh + sd));
  inertiaTensor.set(1, 1, m * (sw + sd));
	inertiaTensor.set(2, 2, m * (sw + sh));

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

Cube::Cube(double side, double mass) :
  Box(side, side, side, mass)
{
}

Cube::~Cube()
{
}
