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

void RigidBody::integrate(double t)
{
  // linear movement
  this->linearMomentum += this->accumulatedForces;
  Vector3 velocity = this->linearMomentum * this->inverseMass;
  this->position += velocity * t;

  // angular movement
  this->angularMomentum += this->accumulatedTorques;
  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  Vector3 angularVelocity = inverseInertia * this->angularMomentum;
  this->orientation += (angularVelocity.toStarMatrix() * this->orientation) * t;

  // normalize the orientation matrix to avoid numerical drift
  this->orientation = this->orientation.normalize();

  this->clearAccumulators();
}

void RigidBody::setPosition(Vector3 position)
{
  this->position = position;
}

void RigidBody::setOrientation(Matrix3 orientation)
{
  this->orientation = orientation;
}

void RigidBody::setFixed(bool fixed)
{
  this->fixed = fixed;
}

void RigidBody::prepare()
{
  if(this->fixed)
  {
    this->inverseMass = 0;
    this->inverseInertiaTensor.reset();
  }
  else
    this->computeInverseInertiaTensor();
}

void RigidBody::computeInverseInertiaTensor()
{

}

