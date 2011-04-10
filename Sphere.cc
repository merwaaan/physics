#include "Sphere.h"

#include <GL/glut.h>

#include "CustomRigidBody.h"
#include "Display.h"
#include "Engine.h"

extern Display* display_pg;
extern Engine* engine_pg;

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
  if(display_pg->areBoundingBoxesDrawn())
    glutWireCube(this->radius * 2);

  glPopMatrix();
}

/**
 * Double-dispatch
 */
std::vector<Contact> Sphere::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

std::vector<Contact> Sphere::isCollidingWith(Sphere* s_p, double dt)
{
  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
 
	// no collision
  if(distance > radii)
  {
    std::cout << "NO CONTACT DETECTED" << std::endl;
    std::vector<Contact> contacts;

    return contacts;
  }

	// collision

  std::cout << "CONTACT DETECTED" << std::endl;
	std::cout << "(separating bodies)" << std::endl;

	double sdt = -dt / 10;

	this->reverseTime();
	s_p->reverseTime();
	
	while(distance < radii)
	{
		std::cout << "distance " << distance << std::endl;
    std::cout << "going backward " << sdt << "ms" << std::endl;

	  engine_pg->applyEnvironmentalForces(this, sdt);
	  engine_pg->applyEnvironmentalForces(s_p, sdt);

		this->integrate(sdt);
    s_p->integrate(sdt);

		distance = (this->position - s_p->position).length();
	}

	this->reverseTime();
	s_p->reverseTime();

  return this->resolveInterPenetration(s_p, dt);
}

std::vector<Contact> Sphere::resolveInterPenetration(Sphere* s_p, double dt)
{
	if(dt < 0.0000000001)
		exit(0);
  
	double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
	int interPenetration = distance < radii;

	std::cout << "dist=" << distance << " radii=" << radii << " ip=" << interPenetration << std::endl;

  if(distance > radii + 0.01)
  {
		double sdt = dt / 2;

    std::cout << "going forward " << sdt << "ms" << std::endl;

		engine_pg->applyEnvironmentalForces(this, sdt);
		engine_pg->applyEnvironmentalForces(s_p, sdt);
    
		this->integrate(sdt);
    s_p->integrate(sdt);
    
    return this->resolveInterPenetration(s_p, sdt);
  }
  else if(distance < radii)
  {
		double sdt = dt / 2;

    std::cout << "going backward " << sdt << "ms" << std::endl;

	  engine_pg->applyEnvironmentalForces(this, sdt);
	  engine_pg->applyEnvironmentalForces(s_p, sdt);

    this->integrateBackward(sdt);
    s_p->integrateBackward(sdt);

    return this->resolveInterPenetration(s_p, dt / 2);
  }

  Contact contact;
  contact.a = this;
  contact.b = s_p;
  contact.position = this->position + (s_p->position - this->position) * (this->radius / radii);
  contact.normal = (s_p->position - this->position).normalize();

  std::vector<Contact> contacts;
  contacts.push_back(contact);

  // recompute auxiliary quantites as they could have been
  // corrupted during the binary search
  contacts[0].a->computeAuxiliaryQuantities();
  contacts[0].b->computeAuxiliaryQuantities();

  return contacts;
}

std::vector<Contact> Sphere::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

/**
 * Return the radius of the sphere
 */
double Sphere::getRadius() const
{
  return this->radius;
}
