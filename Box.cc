#include "Box.h"

Box::Box(double width, double height, double depth) :
  width(width),
  height(height),
  depth(depth)
{
	double hw = width / 2;
	double hh = height / 2;
	double hd = depth / 2;

  this->addVertex(0, -hw, hh, hd, 1);
  this->addVertex(1, -hw, -hh, hd, 1);
  this->addVertex(2, hw, -hh, hd, 1);
  this->addVertex(3, hw, hh, hd, 1);
  this->addVertex(4, -hw, hh, -hd, 1);
  this->addVertex(5, -hw, -hh, -hd, 1);
  this->addVertex(6, hw, -hh, -hd, 1);
  this->addVertex(7, hw, hh, -hd, 1);
  
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

	double sh = this->height * this->height;
	double sw = this->width * this->width;
	double sd = this->depth * this->depth;

  inertiaTensor.set(0, 0, 1 / this->inverseMass / 12 * (sh + sd));
  inertiaTensor.set(1, 1, 1 / this->inverseMass / 12 * (sw + sd));
  inertiaTensor.set(2, 2, 1 / this->inverseMass / 12 * (sw + sh));

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

Cube::Cube(double side) :
  Box(side, side, side)
{
}

Cube::~Cube()
{
}
