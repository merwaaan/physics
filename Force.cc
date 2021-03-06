#include "Force.h"

Force::Force(Vector3 force) :
  force(force)
{
}

Force::~Force()
{
}

CenterForce::CenterForce(Vector3 force) :
  Force(force)
{
}

CenterForce::~CenterForce()
{
}

void CenterForce::apply(RigidBody* rb_p, double dt)
{
  rb_p->applyCenterForce(this->force, dt);
}

OffCenterForce::OffCenterForce(Vector3 force, Vector3 poa) :
  Force(force),
  pointOfApplication(poa)
{
}

OffCenterForce::~OffCenterForce()
{
}

void OffCenterForce::apply(RigidBody* rb_p, double dt)
{
  rb_p->applyOffCenterForce(this->force, this->pointOfApplication, dt);
}


Gravity::Gravity(Vector3 force) :
  CenterForce(force)
{
}

Gravity::~Gravity()
{
}

void Gravity::apply(RigidBody* rb_p, double dt)
{
	Vector3 g = (1/rb_p->getInverseMass()) * this->force;
  rb_p->applyCenterForce(g, dt);
}
