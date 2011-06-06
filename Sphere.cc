#include "Sphere.h"

#include <GL/glut.h>

#include "CustomRigidBody.h"
#include "Display.h"
#include "Engine.h"

extern Engine* E;

Sphere::Sphere(double radius) :
  radius(radius)
{
}

Sphere::~Sphere()
{
}

void Sphere::prepare()
{
  if(this->fixed)
  {
    this->inverseMass = 0;
    this->inverseInertiaTensor.reset();
  }
  else
  {
    this->inverseMass = 1;
    this->computeInverseInertiaTensor();
  }

  this->computeBoundingBox();
}

/**
 * Compute the inverse inertia tensor
 */
void Sphere::computeInverseInertiaTensor()
{
  double i = (2.0 / 5) * (1 / this->inverseMass) * this->radius * this->radius;
  
  Matrix3 inertiaTensor;
  inertiaTensor.set(0, 0, i);
  inertiaTensor.set(1, 1, i);
  inertiaTensor.set(2, 2, i);

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

/**
 * Compute the bounding box
 */
void Sphere::computeBoundingBox()
{
  this->boundingBox.a = this->position + Vector3(-this->radius, -this->radius, -this->radius);
  this->boundingBox.b = this->position + Vector3(this->radius, this->radius, this->radius);
}

void Sphere::integrate(double dt)
{
  RigidBody::integrate(dt);

  this->computeBoundingBox();
}

void Sphere::draw()
{
  glPushMatrix();

  glTranslatef(this->position.X(), this->position.Y(), this->position.Z());
  glutSolidSphere(this->radius, 20, 20);

  // draw the bounding box
  if(E->areBoundingBoxesDrawn())
    glutWireCube(this->radius * 2);

  glPopMatrix();
}

std::vector<Contact> Sphere::getContacts(RigidBody* rb_p)
{
	return this->getContacts(rb_p);
}

std::vector<Contact> Sphere::getContacts(Sphere* s_p)
{
	std::vector<Contact> contacts;

	Vector3 from1to2 = s_p->getPosition() - this->position;
	Vector3 closest1 = this->position + from1to2.normalize() * this->radius;
	Vector3 closest2 = s_p->getPosition() + (-1 * from1to2).normalize() * s_p->getRadius();

	if((closest1 - closest2).length() < E->getTolerance())
	{
		Contact c;
		c.a = this;
		c.b = s_p;
		c.position = closest1 + (closest2 - closest1) / 2;
		c.normal = from1to2.normalize();

		contacts.push_back(c);
	}

	return contacts;
}

std::vector<Contact> Sphere::getContacts(CustomRigidBody* rb_p)
{
	return rb_p->getContacts(this);
}

Vector3 Sphere::getSupportPoint(Vector3 direction)
{
	return this->position + direction.normalize() * this->radius;
}
