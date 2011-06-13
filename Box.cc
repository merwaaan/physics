#include "Box.h"

Box::Box(double width, double height, double depth, double mass) :
  width(width),
  height(height),
  depth(depth)
{
	double hw = width / 2;
	double hh = height / 2;
	double hd = depth / 2;

	double m = mass/8;

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

Box* Box::copy()
{
	Box* copy_p = new Box(this->width, this->height, this->depth, 1/this->inverseMass);

	copy_p->inverseMass = this->inverseMass;
	copy_p->inverseInertiaTensor = this->inverseInertiaTensor;

	copy_p->position = this->position;
	copy_p->linearMomentum = this->linearMomentum;
	copy_p->accumulatedForces = this->accumulatedForces;

	copy_p->orientation = this->orientation;
	copy_p->angularMomentum = this->angularMomentum;
	copy_p->accumulatedTorques = this->accumulatedTorques;

	copy_p->boundingBox = this->boundingBox;

	copy_p->linearVelocity = this->linearVelocity;
	copy_p->angularVelocity = this->angularVelocity;

	copy_p->restitution = this->restitution;
	copy_p->friction = this->friction;

	copy_p->fixed = this->fixed;

	return copy_p;
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

Cube::Cube(double side, double mass) :
  Box(side, side, side, mass)
{
}

Cube::~Cube()
{
}
