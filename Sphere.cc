#include "Sphere.h"

#include <GL/glut.h>

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

void Sphere::computeInverseInertiaTensor()
{
  double i = (2.0 / 5) * (1 / this->inverseMass) * this->radius * this->radius;
  
  Matrix3 inertiaTensor;
  inertiaTensor.set(0, 0, i);
  inertiaTensor.set(1, 1, i);
  inertiaTensor.set(2, 2, i);

  this->inverseInertiaTensor = inertiaTensor.inverse();
}

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

void Sphere::integrate2(double dt)
{
  RigidBody::integrate2(dt);
  
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

Contact* Sphere::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

Contact* Sphere::isCollidingWith(Sphere* s_p, double dt)
{
  double tolerance = 0.01;

  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
 
  if(distance > radii + tolerance)
  {
    std::cout << "NO CONTACT" << std::endl;

    return NULL;
  }
  else
    return this->resolveInterPenetration(s_p, dt, tolerance);
}

Contact* Sphere::resolveInterPenetration(Sphere* s_p, double dt, double tolerance)
{
  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
 
  if(distance > radii + tolerance)
  {
    std::cout << "OUTSIDE (going on " << dt / 2 << ")" << std::endl;

    // integrate forward in time in order to determine the real contact point
    this->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    s_p->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    this->integrate(dt / 2);
    s_p->integrate(dt / 2);
    
    return this->resolveInterPenetration(s_p, dt / 2, tolerance);
  }
  else if(distance < radii - tolerance)
  {
    std::cout << "INSIDE (going back " << dt / 2 << ")" << std::endl;

    // integrate backward in time in order to determine the real contact point
    this->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    s_p->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    engine_pg->reverseTime();
    this->integrate(-dt / 2);
    s_p->integrate(-dt / 2);
    engine_pg->reverseTime();

    return this->resolveInterPenetration(s_p, dt / 2, tolerance);
  }

  std::cout << "SURFACE CONTACT" << std::endl;

  Contact* contact_p = new Contact;
  contact_p->a = this;
  contact_p->b = s_p;
  contact_p->position = this->position + (s_p->position - this->position) * (this->radius / radii);
  contact_p->normal = (s_p->position - this->position).normalize();

  return contact_p;
}

Contact* Sphere::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  return NULL;
}

