#include "RigidBody.h"

RigidBody::RigidBody() :
  fixed(false)
{
  // initial orientation (aligned with the axis)
  this->orientation(0, 0, 1);
  this->orientation(1, 1, 1);
  this->orientation(2, 2, 1);
}

RigidBody::~RigidBody()
{
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb)
{
  os << "position " << rb.position << std::endl;
  os << "linear momentum " << rb.linearMomentum << std::endl;
  os << "orientation " << std::endl << rb.orientation;
  os << "angular momentum " << rb.angularMomentum << std::endl;

  return os;
}

void RigidBody::clearAccumulators()
{
  this->accumulatedForces.reset();
  this->accumulatedTorques.reset();
}

void RigidBody::applyCenterForce(Vector3 force)
{
  this->accumulatedForces += force;
}

void RigidBody::applyOffCenterForce(Vector3 force, Vector3 poa)
{
  this->accumulatedForces += force;
  this->accumulatedTorques += poa ^ force;
}

void RigidBody::integrate(double dt)
{
  DerivativeState start;

  DerivativeState k1 = this->evaluate(0, start);
  DerivativeState k2 = this->evaluate(dt * 0.5, k1);
  DerivativeState k3 = this->evaluate(dt * 0.5, k2);
  DerivativeState k4 = this->evaluate(dt, k3);

  this->position +=
    (dt < 0 ? -1 : 1) *
    dt / 6 *
    (k1.deltaPosition + 2 * k2.deltaPosition + 2 * k3.deltaPosition + k4.deltaPosition);

  this->linearMomentum +=
    (dt < 0 ? -1 : 1) *
    dt / 6 *
    (k1.deltaLinearMomentum + 2 * k2.deltaLinearMomentum + 2 * k3.deltaLinearMomentum + k4.deltaLinearMomentum);

  /*// linear movement
  this->linearMomentum += this->accumulatedForces;
  Vector3 velocity = this->linearMomentum * this->inverseMass * dt;
  this->position += velocity * dt;

  // angular movement
  this->angularMomentum += this->accumulatedTorques;
  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  Vector3 angularVelocity = inverseInertia * this->angularMomentum * dt;
  this->orientation += (angularVelocity.toStarMatrix() * this->orientation) * dt;

  // normalize the orientation matrix to avoid numerical drift
  this->orientation = this->orientation.normalize();*/
}

void RigidBody::integrate2(double dt)
{
  Vector3 velocity = this->linearMomentum * this->inverseMass * dt;
  this->position += velocity * dt;
  
  this->linearMomentum += this->accumulatedForces;
}

DerivativeState RigidBody::evaluate(double dt, DerivativeState ds)
{
  // current rigid body state
  Vector3 linearMomentum = this->linearMomentum + ds.deltaLinearMomentum * dt * (dt < 0 ? -1 : 1);

  DerivativeState ds2;
  ds2.deltaPosition = linearMomentum * this->inverseMass;
  ds2.deltaLinearMomentum = this->accumulatedForces;

  return ds2;
}

bool RigidBody::isBoundingBoxCollidingWith(RigidBody* rb_p)
{
  BoundingBox b1 = this->boundingBox;
  BoundingBox b2 = rb_p->getBoundingBox();

  if(
    b1.a.X() > b2.b.X() || b1.b.X() < b2.a.X() ||
    b1.a.Y() > b2.b.Y() || b1.b.Y() < b2.a.Y() ||
    b1.a.Z() > b2.b.Z() || b1.b.Z() < b2.a.Z())
    return false;

  return true;
}

BoundingBox RigidBody::getBoundingBox()
{
  return this->boundingBox;
}

void RigidBody::setPosition(Vector3 position)
{
  this->position = position;
}

void RigidBody::setPosition(double x, double y, double z)
{
  this->position = Vector3(x, y, z);
}

void RigidBody::setOrientation(Matrix3 orientation)
{
  this->orientation = orientation;
}

void RigidBody::setFixed(bool fixed)
{
  this->fixed = fixed;
}

Vector3 RigidBody::getVelocity(double dt)
{
  return this->linearMomentum * this->inverseMass * dt;
}

