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

Contact* Sphere::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

Contact* Sphere::isCollidingWith(Sphere* s_p, double dt)
{
  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
  double tolerance = 0.3;
 
  if(distance > radii + tolerance)
  {
    std::cout << "NO CONTACT" << std::endl;

    return NULL;
  }
  else
    return this->resolveInterPenetration(s_p, dt / 2, tolerance);
}

Contact* Sphere::resolveInterPenetration(Sphere* s_p, double dt, double tolerance)
{
  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
 
  if(distance > radii + tolerance)
  {
    std::cout << "OUTSIDE (going on " << dt << ")" << std::endl;
    std::cout << this->position << this->linearMomentum << std::endl;
    
    this->integrate(dt);
    s_p->integrate(dt);

    return this->resolveInterPenetration(s_p, dt / 2, tolerance);
  }
  else if(distance < radii - tolerance)
  {
    std::cout << "INSIDE (going back " << dt << ")" << std::endl;
    std::cout << this->position << this->linearMomentum << std::endl;

    engine_pg->reverseTime();
    this->integrate(-dt);
    s_p->integrate(-dt);
    engine_pg->reverseTime();

    return this->resolveInterPenetration(s_p, dt / 2, tolerance);
  }

  std::cout << "SURFACE CONTACT" << std::endl;
  std::cout << this->position << this->linearMomentum << std::endl;

  Contact* contact_p = new Contact;
  contact_p->a = this;
  contact_p->b = s_p;
  contact_p->position = this->position + (this->position - s_p->position) * (this->radius / radii);
  contact_p->normal = (s_p->position - this->position).normalize();

  return contact_p;
}

Contact* Sphere::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  return NULL;
}

