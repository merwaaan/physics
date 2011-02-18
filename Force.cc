#include "Force.h"

Force::Force() :
  on(true)
{
}

Force::~Force()
{
}

Gravity::Gravity(Vector3 force) :
  force(force)
{
}

Gravity::~Gravity()
{
}

void Gravity::apply(RigidBody* rb_p)
{
  rb_p->applyForce(this->force);
}

AirFriction::AirFriction()
{

}

AirFriction::~AirFriction()
{
}

