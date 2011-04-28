#include "RigidBody.h"

#include "Engine.h"

extern Engine* engine_pg;

RigidBody::RigidBody() :
	restitution(0.8),
	friction(0.9),
  fixed(false)
{
  // The initial orientation is aligned with the axis.
  this->orientation.set(0, 0, 1);
  this->orientation.set(1, 1, 1);
  this->orientation.set(2, 2, 1);
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
	if(this->fixed)
		return;

  this->accumulatedForces += force * dt * (dt < 0 ? -1 : 1);
}

void RigidBody::applyOffCenterForce(Vector3 force, double dt, Vector3 poa)
{
	if(this->fixed)
		return;

	this->accumulatedForces += (force * dt) * (dt < 0 ? -1 : 1);
  this->accumulatedTorques += (poa - this->position) ^ (force * dt) * (dt < 0 ? -1 : 1);
}

void RigidBody::computeAuxiliaryQuantities()
{
	this->linearVelocity = this->linearMomentum * this->inverseMass;

  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  this->angularVelocity = inverseInertia * this->angularMomentum;
}

void RigidBody::integrate(double dt)
{
  // Short-circuit integration if the body is fixed.
  if(this->fixed)
    return;

	// LINEAR COMPONENT

	if(dt > 0)
		this->linearMomentum += this->accumulatedForces;

  Vector3 velocity = this->linearMomentum * this->inverseMass;
  this->position += velocity * dt * (dt < 0 ? -1 : 1);

	if(dt < 0)
		this->linearMomentum += this->accumulatedForces;

	// ANGULAR COMPONENT

	if(dt > 0)
		this->angularMomentum += this->accumulatedTorques;

  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  Vector3 angularVelocity = inverseInertia * this->angularMomentum;
  this->orientation += (angularVelocity.skew() * this->orientation) * dt * (dt < 0 ? -1 : 1);

	if(dt < 0)
		this->angularMomentum += this->accumulatedTorques;

	// Reorthogonalize and normalize the orientation matrix to avoid numerical drift.
  this->orientation = this->orientation.orthogonalize();
  this->orientation = this->orientation.normalize();

  // Clear the forces accumulated during the last frame.
  this->clearAccumulators();

  // Cache the auxiliary quantities.
  this->computeAuxiliaryQuantities();
}

void RigidBody::integrateBackward(double dt)
{
	if(this->fixed)
		return;

	this->reverseTime();
	this->integrate(-dt);
	this->reverseTime();
}

void RigidBody::reverseTime()
{
	this->linearMomentum *= -1;
	this->angularMomentum *= -1;
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

Vector3 RigidBody::getPosition() const
{
  return this->position;
}

Vector3 RigidBody::getVelocity() const
{
  return this->linearMomentum * this->inverseMass;
}

Vector3 RigidBody::getVelocity(const Vector3& point) const
{
	return this->linearVelocity + (this->angularVelocity ^ (point - this->position));
}

void RigidBody::setRestitution(double restitution)
{
	this->restitution = restitution;
}

double RigidBody::getRestitution() const
{
	return this->restitution;
}

void RigidBody::setFriction(double friction)
{
	this->friction = friction;
}

double RigidBody::getFriction() const
{
	return this->friction;
}
