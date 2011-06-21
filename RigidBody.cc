#include "RigidBody.h"

#include "Engine.h"

extern Engine* E;

RigidBody::RigidBody() :
	restitution(0.2),
  fixed(false),
	sleeping(false),
	kineticEnergyLowFor(0),
	couldSleep(false)
{
  // The initial orientation is aligned with the axis.
  this->orientation.set(0, 0, 1);
  this->orientation.set(1, 1, 1);
  this->orientation.set(2, 2, 1);

	this->color = new float[3];
	this->color[0] = this->color[1] = this->color[2] = 0.5;
}

RigidBody::~RigidBody()
{
	delete[] this->color;
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
	if(!this->isActive())
		return;

  this->accumulatedForces += force * dt * (dt < 0 ? -1 : 1);
}

void RigidBody::applyOffCenterForce(Vector3 force, Vector3 poa, double dt)
{
	if(!this->isActive())
		return;

	this->accumulatedForces += force * dt * (dt < 0 ? -1 : 1);
  this->accumulatedTorques += (poa - this->position) ^ (force * dt) * (dt < 0 ? -1 : 1);
}

void RigidBody::integrate(double dt)
{
  // Short-circuit integration if the body is fixed or asleep.
  if(!this->isActive())
    return;

	// Check if switching to sleep is necesary.
	this->handleSleep();

	// LINEAR COMPONENT

	if(dt > 0)
		this->linearMomentum += this->accumulatedForces;

  this->linearVelocity = this->linearMomentum * this->getInverseMass();
  this->position += this->linearVelocity * dt * (dt < 0 ? -1 : 1);

	if(dt < 0)
		this->linearMomentum += this->accumulatedForces;

	// ANGULAR COMPONENT

	if(dt > 0)
		this->angularMomentum += this->accumulatedTorques;

  Matrix3 inverseInertia = this->orientation * this->inverseInertiaTensor * this->orientation.transpose();
  this->angularVelocity = inverseInertia * this->angularMomentum;
  this->orientation += (this->angularVelocity.skew() * this->orientation) * dt * (dt < 0 ? -1 : 1);

	if(dt < 0)
		this->angularMomentum += this->accumulatedTorques;

	// Reorthogonalize and normalize the orientation matrix to avoid numerical drift.
  this->orientation = this->orientation.orthogonalize();
  this->orientation = this->orientation.normalize();

  // Clear the forces accumulated during the last frame.
  this->clearAccumulators();
}

void RigidBody::integrateBackward(double dt)
{
	if(!this->isActive())
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

void RigidBody::handleSleep()
{
	if(this->couldSleep == true && this->getKineticEnergy() < 3)
	{
		++this->kineticEnergyLowFor;

		if(kineticEnergyLowFor > 3)
			this->setSleeping(true);
	}
	else
	{
		this->kineticEnergyLowFor = 0;
		couldSleep = false;
	}
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
 * Check for a collision with another rigid body.
 */
std::vector<Contact> RigidBody::isCollidingWith(RigidBody* rb_p, double dt)
{
	bool interPenetration;
	Vector3 distance = Geometry::gjkDistance(this, rb_p, &interPenetration);

	std::cout << "distance " << distance << std::endl;

  // Check for a true collision.
	if(!interPenetration)
    return std::vector<Contact>();

  return this->resolveInterPenetration(rb_p, dt, FORWARD, dt);
}

/**
 * Determine the exact contact point by integrating backward in time.
 */
std::vector<Contact> RigidBody::resolveInterPenetration(RigidBody* rb_p, double dt, Dir direction, double TOI)
{
	//std::cout << "TOI " << TOI << std::endl;
  // Compute the distance between the two bodies.
	bool interPenetration;
  Vector3 distance = Geometry::gjkDistance(this, rb_p, &interPenetration);

  std::cout << "distance " << distance.length() << std::endl;

  // If the bodies are too far apart, integrate forward in time.
  if(!interPenetration && distance.length() > E->getCollisionTolerance())
  {
		double sdt = direction != FORWARD ? dt/2 : dt;
		direction = FORWARD;

	  std::cout << "going forward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrate(sdt);
    rb_p->integrate(sdt);

    //std::cout << *this << std::endl;

    return this->resolveInterPenetration(rb_p, sdt, direction, TOI+sdt);
  }
  // Else if the bodies are inter-penetrating, integrate backward in time.
  else if(interPenetration)
  {
		double sdt = direction != BACKWARD ? dt/2 : dt;
		direction = BACKWARD;

	  std::cout << "going backward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrateBackward(sdt);
    rb_p->integrateBackward(sdt);

//		std::cout << *this << std::endl;

    return this->resolveInterPenetration(rb_p, sdt, direction, TOI-sdt);
  }

  // If bodies are within the tolerance area, compute the real contact points.
	std::vector<Contact> contacts = this->getContacts(rb_p);

	if(contacts.size() > 0)
		// Append the time of impact to the contacts.
		for(int i = 0; i < contacts.size(); ++i)
			contacts[i].TOI = TOI;

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

double RigidBody::getKineticEnergy()
{
	return (1/this->getInverseMass()) * pow(this->getVelocity().length(), 2) / 2;
}

void RigidBody::setColor(int r, int g, int b)
{
	this->color[0] = r/255.;
	this->color[1] = g/255.;
	this->color[2] = b/255.;
}
