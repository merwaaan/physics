#include "Sphere.h"

#include <GL/glut.h>

#include "CustomRigidBody.h"
#include "Display.h"
#include "Engine.h"

extern Engine* E;

Sphere::Sphere(double radius) :
  radius(radius)
{
	this->type = SPHERE;
}

Sphere::~Sphere()
{
}

Sphere* Sphere::copy()
{
	Sphere* copy_p = new Sphere(this->radius);

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
	if(rb_p->type == SPHERE)
		return this->getContacts((Sphere*)rb_p);

	return this->getContacts((CustomRigidBody*)rb_p);
}

std::vector<Contact> Sphere::getContacts(Sphere* s_p)
{
	std::vector<Contact> contacts;

	Vector3 from1to2 = s_p->getPosition() - this->position;
	Vector3 closest1 = this->position + from1to2.normalize() * this->radius;
	Vector3 closest2 = s_p->getPosition() + (-1 * from1to2).normalize() * s_p->getRadius();

	//if((closest1 - closest2).length() < E->getTolerance())
	if(closest1 == closest2)
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
	std::vector<Contact> contacts;

	Vector3 dirSphereToCustom = rb_p->getPosition() - this->position;
	Vector3 closest = this->position + this->radius * dirSphereToCustom.normalize();

  for(int i = 0; i < rb_p->structure.polygons.size(); ++i)
	{
		Polygon face = rb_p->structure.polygons[i].getPolygon();

		double distance;
		Vector3 point = Geometry::closestPointOfPolygon(closest, face, &distance);

		if(distance < 0.1)
		{
			Contact contact;
			
			contact.a = rb_p;
			contact.b = this;
			contact.position = point;
			contact.normal = (closest - point).normalize();
			
			contacts.push_back(contact);  
		}
	}

  return contacts;
}

Vector3 Sphere::getSupportPoint(Vector3 direction)
{
	return this->position + direction.normalize() * this->radius;
}
