#include "Box.h"

Box::Box(double width, double height, double depth) :
  width(width),
  height(height),
  depth(depth)
{
  this->addVertex(0, -width / 2, height / 2, depth / 2, 1);
  this->addVertex(1, -width / 2, -height / 2, depth / 2, 1);
  this->addVertex(2, width / 2, -height / 2, depth / 2, 1);
  this->addVertex(3, width / 2, height / 2, depth / 2, 1);
  this->addVertex(4, -width / 2, height / 2, -depth / 2, 1);
  this->addVertex(5, -width / 2, -height / 2, -depth / 2, 1);
  this->addVertex(6, width / 2, -height / 2, -depth / 2, 1);
  this->addVertex(7, width / 2, height / 2, -depth / 2, 1);
  
  this->addPolygon(4, (int[]){0, 1, 2, 3}); // front
  this->addPolygon(4, (int[]){7, 6, 5, 4}); // back
  this->addPolygon(4, (int[]){0, 3, 7, 4}); // top
  this->addPolygon(4, (int[]){1, 2, 6, 5}); // bottom
  this->addPolygon(4, (int[]){4, 5, 1, 0}); // left
  this->addPolygon(4, (int[]){7, 3, 2, 6}); // right
}

Box::~Box()
{
}

void Box::computeInverseInertiaTensor()
{
  Matrix3 inertiaTensor;
  inertiaTensor(0, 0, 1 / this->inverseMass / 12 * (this->height * this->height + this->depth * this->depth));
  inertiaTensor(1, 1, 1 / this->inverseMass / 12 * (this->width * this->width + this->depth * this->depth));
  inertiaTensor(2, 2, 1 / this->inverseMass / 12 * (this->width * this->width + this->height * this->height));

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

Cube::Cube(double side) :
  Box(side, side, side)
{
}

Cube::~Cube()
{
}

