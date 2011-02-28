#include "Sphere.h"

#include <GL/glut.h>

#include "Display.h"

extern Display* display_pg;

Sphere::Sphere(double radius) :
  radius(radius)
{
}

Sphere::~Sphere()
{
}

void Sphere::prepare()
{
  this->inverseMass = 1;
  this->computeInverseInertiaTensor();
}

void Sphere::computeInverseInertiaTensor()
{
  Matrix3 inertiaTensor;
  double i = (2.0 / 5) * (1 / this->inverseMass) * this->radius * this->radius;
  inertiaTensor(0, 0, i);
  inertiaTensor(1, 1, i);
  inertiaTensor(2, 2, i);

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

void Sphere::computeBoundingBox()
{
  this->boundingBox.a = this->position + Vector3(-this->radius, -this->radius, -this->radius);
  this->boundingBox.b = this->position + Vector3(this->radius, this->radius, this->radius);
}

void Sphere::integrate(double t)
{
  RigidBody::integrate(t);

  this->computeBoundingBox();
}

void Sphere::draw()
{
  glPushMatrix();

  glTranslatef(this->position.X(), this->position.Y(), this->position.Z());
  glutSolidSphere(this->radius, 20, 20);

  // draw the bounding box
  if(display_pg->areBoundingBoxesDrawn())
    glutWireCube(this->radius * 2);

  glPopMatrix();
}

