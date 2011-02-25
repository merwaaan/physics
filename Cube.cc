#include "Cube.h"

Cube::Cube(double side) :
  side(side),
  h(side / 2)
{
  this->addVertex(0, -h, h, h, 1);
  this->addVertex(1, -h, -h, h, 1);
  this->addVertex(2, h, -h, h, 1);
  this->addVertex(3, h, h, h, 1);
  this->addVertex(4, -h, h, -h, 1);
  this->addVertex(5, -h, -h, -h, 1);
  this->addVertex(6, h, -h, -h, 1);
  this->addVertex(7, h, h, -h, 1);
  
  this->addPolygon(4, (int[]){0, 1, 2, 3}); // front
  this->addPolygon(4, (int[]){7, 6, 5, 4}); // back
  this->addPolygon(4, (int[]){0, 3, 7, 4}); // top
  this->addPolygon(4, (int[]){1, 2, 6, 5}); // bottom
  this->addPolygon(4, (int[]){4, 5, 1, 0}); // left
  this->addPolygon(4, (int[]){7, 3, 2, 6}); // right
}

Cube::~Cube()
{
}

void Cube::computeInverseInertiaTensor()
{
  Matrix3 inertiaTensor;
  inertiaTensor(0, 0, 1 / this->structure.inverseMass / 12 * (this->side * this->side + this->side * this->side));
  inertiaTensor(1, 1, 1 / this->structure.inverseMass / 12 * (this->side * this->side + this->side * this->side));
  inertiaTensor(2, 2, 1 / this->structure.inverseMass / 12 * (this->side * this->side + this->side * this->side));

  this->structure.inverseInertiaTensor = inertiaTensor.inverse();
}

