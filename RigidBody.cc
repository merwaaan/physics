#include "RigidBody.h"

#include "Engine.h"

extern Engine* E;

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

void RigidBody::applyOffCenterForce(Vector3 force, Vector3 poa, double dt)
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

/**
 * Check for a collision with another rigid body
 */
std::vector<Contact> RigidBody::isCollidingWith(RigidBody* rb_p, double dt)
{
	bool ip;
	Vector3 dist = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &ip);
	std::cout << "ip " << ip << " dist " << dist << std::endl;

  // Check for a true collision.
	if(!ip)
  {
	  std::cout << "Narrow-phase detection : NO " << std::endl;

    return std::vector<Contact>();
  }
 
	std::cout << "Narrow-phase detection : YES " << std::endl;

	int sdiv = 5;
	double sdt = -dt / sdiv;

	this->reverseTime();
	rb_p->reverseTime();

	// Separate the bodies to speed up the penetration resolution.
	int backtracks = 0;
	for(int i = 0; i < sdiv; ++i)
	{
    std::cout << "going backward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

		this->integrate(sdt);
		rb_p->integrate(sdt);

		++backtracks;
		std::cout << rb_p->position << std::endl;

    // Exit the loop when no more interpenetration.
		Vector3 dist = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &ip);
		std::cout << "ip " << ip << " dist " << dist << std::endl;
		if(!ip)
			break;
	}

	this->reverseTime();
	rb_p->reverseTime();

	// Apply an emergency push if the bodies are stuck.
/*	if(!Geometry::findSeparatingPlane(this, rb_p))
	{
		std::cout << "push!!!" << std::endl;
		E->emergencyPush(this, rb_p);
	}
*/
	std::cout << "NEXT STEP" << std::endl;
  return this->resolveInterPenetration(rb_p, sdt * backtracks);
}

/**
 * Determine the exact contact point by integrating backward in time
 */
std::vector<Contact> RigidBody::resolveInterPenetration(RigidBody* rb_p, double dt)
{
  // compute the distance between the two bodies
	bool interPenetration;
  Vector3 distance = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &interPenetration);

  std::cout << "d = " << distance.length() << " ip = " << interPenetration << std::endl;

  // if the bodies are too far apart, integrate forward in time
  if(!interPenetration && distance.length() > E->getTolerance())
  {
		double sdt = dt / 2;

	  std::cout << "going forward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrate(sdt);
    rb_p->integrate(sdt);

    std::cout << *rb_p << std::endl;

    return this->resolveInterPenetration(rb_p, sdt);
  }
  // else if the bodies are inter-penetrating, integrate backward in time
  else if(interPenetration)
  {
		double sdt = dt / 2;

	  std::cout << "going backward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrateBackward(sdt);
    rb_p->integrateBackward(sdt);

		std::cout << *rb_p << std::endl;

    return this->resolveInterPenetration(rb_p, sdt);
  }

  // If bodies are within the tolerance area, compute the real contact points.
	std::vector<Contact> contacts = this->getContacts(rb_p);

	// recompute auxiliary quantities as they could have been
	// corrupted during the binary search
	if(contacts.size() > 0)
	{
		contacts[0].a->computeAuxiliaryQuantities();
		contacts[0].b->computeAuxiliaryQuantities();
	}

	return contacts;
}

BoundingBox RigidBody::getBoundingBox()
{
  return this->boundingBox;
}

Vector3 RigidBody::getVelocity() const
{
  return this->linearMomentum * this->inverseMass;
}

Vector3 RigidBody::getVelocity(const Vector3& point) const
{
	return this->linearVelocity + (this->angularVelocity ^ (point - this->position));
}
