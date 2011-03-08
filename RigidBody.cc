#include "RigidBody.h"

#include "Engine.h"

extern Engine* engine_pg;

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
  os << "position : " << rb.position << std::endl;
  os << "linear momentum : " << rb.linearMomentum << std::endl;
  os << "orientation : " << std::endl << rb.orientation;
  os << "angular momentum : " << rb.angularMomentum << std::endl;

  return os;
}

void RigidBody::clearAccumulators()
{
  this->accumulatedForces.reset();
  this->accumulatedTorques.reset();
}

void RigidBody::applyCenterForce(Vector3 force, double dt)
{
  this->accumulatedForces += force * dt;
}

void RigidBody::applyOffCenterForce(Vector3 force, double dt, Vector3 poa)
{
  this->accumulatedForces += force * dt;
  this->accumulatedTorques += poa ^ (force * dt);
}

void RigidBody::integrate(double dt)
{
  // we don't need to integrate fixed bodies
  if(this->inverseMass == 0)
    return;

  DerivativeState start;

  DerivativeState k1 = this->evaluate(dt, 0, start);
  DerivativeState k2 = this->evaluate(dt, dt * 0.5, k1);
  DerivativeState k3 = this->evaluate(dt, dt * 0.5, k2);
  DerivativeState k4 = this->evaluate(dt, dt, k3);

  this->position +=
    (dt < 0 ? -1 : 1) *
    1.0 / 6 *
    (k1.deltaPosition + 2 * k2.deltaPosition + 2 * k3.deltaPosition + k4.deltaPosition);
  
  this->linearMomentum +=
    (dt < 0 ? -1 : 1) *
    1.0 / 6 *
    (k1.deltaLinearMomentum + 2 * k2.deltaLinearMomentum + 2 * k3.deltaLinearMomentum + k4.deltaLinearMomentum);

  this->orientation +=
    (dt < 0 ? -1 : 1) *
    1.0 / 6 *
	  (k1.deltaOrientation + 2 * k2.deltaOrientation + 2 * k3.deltaOrientation + k4.deltaOrientation);

  this->angularMomentum +=
    (dt < 0 ? -1 : 1) *
    1.0 / 6 *
    (k1.deltaAngularMomentum + 2 * k2.deltaAngularMomentum + 2 * k3.deltaAngularMomentum + k4.deltaAngularMomentum);

  // reorthogonalize and normalize the orientation matrix to avoid numerical drift
  this->orientation = this->orientation.orthogonalize();
  this->orientation = this->orientation.normalize();

  // clear the forces accumulated during the last frame
  this->clearAccumulators();

/*
  // linear movement
  this->linearMomentum += this->accumulatedForces;
  Vector3 velocity = this->linearMomentum * this->inverseMass * dt;
  this->position += velocity * dt;

  // angular movement
  this->angularMomentum += this->accumulatedTorques;
  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  Vector3 angularVelocity = inverseInertia * this->angularMomentum * dt;
  this->orientation += (angularVelocity.toStarMatrix() * this->orientation) * dt;
*/
}

void RigidBody::integrate2(double dt)
{
  Vector3 velocity = this->linearMomentum * this->inverseMass * dt;
  this->position += velocity * dt;
  
  this->linearMomentum += this->accumulatedForces;

  this->clearAccumulators();
}

DerivativeState RigidBody::evaluate(double dt, double sdt, DerivativeState ds)
{
  Vector3 linearMomentum = this->linearMomentum + ds.deltaLinearMomentum * (dt >= 0 ? sdt / dt : 1 - sdt / dt);
  Vector3 angularMomentum = this->angularMomentum + ds.deltaAngularMomentum * (dt >= 0 ? sdt / dt : 1 - sdt / dt);

  DerivativeState ds2;

  // linear component
  Vector3 velocity = linearMomentum * this->inverseMass * sdt;
  ds2.deltaPosition = velocity;
  ds2.deltaLinearMomentum = this->accumulatedForces;

  // angular component
  Matrix3 orientation = this->orientation + ds.deltaOrientation;
  Matrix3 inverseInertia = orientation * this->inverseInertiaTensor * orientation.transpose();
  Vector3 angularVelocity = inverseInertia * angularMomentum * sdt;
  ds2.deltaOrientation = angularVelocity.toStarMatrix() * this->orientation;
  ds2.deltaAngularMomentum = this->accumulatedTorques;

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

Vector3 RigidBody::getVelocity()
{
  return this->linearMomentum * this->inverseMass;
}

