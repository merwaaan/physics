#include "Force.h"

Force::Force(Vector3 force) :
  force(force),
  on(true)
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

void CenterForce::apply(RigidBody* rb_p)
{
  rb_p->applyCenterForce(this->force);
}

OffCenterForce::OffCenterForce(Vector3 force, Vector3 poa) :
  Force(force),
  pointOfApplication(poa)
{
}

OffCenterForce::~OffCenterForce()
{
}

void OffCenterForce::apply(RigidBody* rb_p)
{
  rb_p->applyOffCenterForce(this->force, this->pointOfApplication);
}

