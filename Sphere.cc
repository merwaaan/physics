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

bool r;
Contact* Sphere::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

Contact* Sphere::isCollidingWith(Sphere* s_p, double dt)
{
  double distance = (this->position - s_p->position).length();
  double radii = this->radius + s_p->radius;
  double tolerance = 1;

  /**
   *  INTER-PENETRATING CONTACT
   *  we need to determine the real contact point
   */
  //if(distance < radii - tolerance)
    //this->resolveInterPenetration(s_p, -dt);
  /**
   * NO CONTACT
   */
  //else if(distance > radii + tolerance)
    //  return NULL;
  /**
   *  SURFACE CONTACT
   *  we already know the real contact point
   */
  //else
  if(distance < radii)
  {
    Contact* contact_p = new Contact;

    contact_p->a = this;
    contact_p->b = s_p;
    contact_p->position = (this->position - s_p->position) * (1 / this->radius / (this->radius + s_p->radius));
    contact_p->normal = (s_p->position - this->position).normalize();

    std::cout << "SURFACE CONTACT" << std::endl;
    
    return contact_p;
  }

  return NULL;
}

Contact* Sphere::resolveInterPenetration(Sphere* s_p, double dt)
{
  std::cout << "INTER-PENETRATING (moving " << dt << ") " << (this->position - s_p->position).length() << std::endl;

  // copy the two inter-penetrating bodies
  Sphere* this_copy = new Sphere(*this);
  Sphere* s_p_copy = new Sphere(*s_p);

  // integrate the copies backward in time
  this_copy->linearMomentum = this_copy->linearMomentum * -1;
  this_copy->integrate(dt / 2);
  s_p_copy->integrate(dt / 2);
  this_copy->linearMomentum = this_copy->linearMomentum * -1;

std::cout << *this_copy << std::endl;

  // evaluate their previous state
  Contact* contact_p = this_copy->isCollidingWith(s_p_copy, dt);

  // update the original bodies states
  this->position = contact_p->a->position;
  this->linearMomentum = contact_p->a->linearMomentum;
  s_p->position = contact_p->b->position;
  s_p->linearMomentum = contact_p->b->linearMomentum;

  // points to the original bodies but keep the position and normal computed for their integrated states
  contact_p->a = this;
  contact_p->b = s_p;
    
  //delete this_copy;
  //delete s_p_copy;

  return contact_p;
}

Contact* Sphere::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  return NULL;
}

