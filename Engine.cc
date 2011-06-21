#include "Engine.h"

#include <stdlib.h>
#include <sys/time.h>

#include "Geometry.h"
#include "Sphere.h"
#include "Box.h"

Engine* E = NULL;

Engine::Engine(int* argc, char** argv, double timeStep) :
  timeStep(timeStep),
	updateType(IMPLICIT),
	collisionTolerance(0.1),
	geometryTolerance(0.01),
	startingTime(getAbsoluteTime()),
	lastUpdateTime(0),
  display(argc, argv, 600, 600)
{
  E = this;
}

Engine::~Engine()
{
  for(int i = 0; i < this->bodies_p.size(); ++i)
    delete this->bodies_p[i];

	for(int i = 0; i < this->environmentalForces_p.size(); ++i)
		delete this->environmentalForces_p[i];
	
	for(int i = 0; i < this->constraints_p.size(); ++i)
		delete this->constraints_p[i];
}

void Engine::run()
{
	// Prepare bodies.
  for(int i = 0; i < this->bodies_p.size(); ++i)
    this->bodies_p[i]->prepare();

  this->display.run();
}

void Engine::update()
{
	if(this->updateType == IMPLICIT)
		this->updateFixed();
	else if(this->updateType == EXPLICIT)
		this->updateContinuous();
}

void Engine::updateFixed()
{
	// Check for collisions.
	for(int i = 1; i < this->bodies_p.size(); ++i)
		for(int j = 0; j < i; ++j)
		{
			// No need to go further is the two bodies are fixed.
			if(!this->bodies_p[i]->isActive() && !this->bodies_p[j]->isActive())
				continue;
			
			// Broad-phase test.
			if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
			{
				std::cout << "AABB collision between #" << i << " and #" << j << std::endl;
				
				// Narrow-phase test.
				std::vector<Contact> contacts = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);
				
				if(contacts.size() > 0)
				{
					if(contacts[0].a->isSleeping())
						contacts[0].a->setSleeping(false);

					if(contacts[0].b->isSleeping())
						contacts[0].b->setSleeping(false);

					std::cout << "GJK collision detected between #" << i << " and #" << j << std::endl;
					
					// Security check for swapped pointers to the bodies.
					contacts = this->checkContacts(contacts);

					for(int k = 0; k < contacts.size(); ++k)
					{
						Vector3* impulses = this->computeImpulse(contacts[k]);
						contacts[k].a->applyOffCenterForce(impulses[0], contacts[k].position, 1);
						contacts[k].b->applyOffCenterForce(impulses[1], contacts[k].position, 1);
					}
				}
			}
		}

	// Apply constraints.
	this->applyConstraints(1);

	for(int i = 0; i < this->bodies_p.size(); ++i)
	{      
		// Apply external forces.
		for(int j = 0; j < this->environmentalForces_p.size(); ++j)
			this->environmentalForces_p[j]->apply(this->bodies_p[i], this->timeStep);
		
		// Integrate each body.
		this->bodies_p[i]->integrate(this->timeStep);
	}
	
	this->lastUpdateTime += this->timeStep;
	
	//this->cleanUp();
}

void Engine::updateContinuous()
{
	// Predict contacts to come.
	std::vector<Contact> futureContacts = this->predictContacts();
	//for(int i = 0; i < futureContacts.size(); ++i)
	//std::cout << "fc " << futureContacts[i].TOI << std::endl;

  double timeStepToFirstContact = futureContacts.size() > 0 ? futureContacts[0].TOI : this->timeStep;
//timeStepToFirstContact = timeStepToFirstContact < this->timeStep/4 ? this->timeStep/4 : timeStepToFirstContact;

	// Apply constraints.
	// this->applyConstraints(timeStepToFirstContact);

	// Integrate forward in time to the first contact.
	std::cout << "REAL INTEGRATION" << std::endl;
	for(int i = 0; i < this->bodies_p.size(); ++i)
	{      
		// Apply external forces.
		for(int j = 0; j < this->environmentalForces_p.size(); ++j)
			this->environmentalForces_p[j]->apply(this->bodies_p[i], timeStepToFirstContact);

		// Integrate each body.
		this->bodies_p[i]->integrate(timeStepToFirstContact);
	}

	std::cout << "PREDICTED CONTACT HANDLING" << std::endl;
	// Handle predicted contacts.
	for(int i = 0; i < futureContacts.size(); ++i)
	{
		if(futureContacts[i].TOI > timeStepToFirstContact)
			break;

		Vector3* impulses = this->computeImpulse(futureContacts[i]);

		futureContacts[i].a->applyOffCenterForce(impulses[0], futureContacts[i].position, 1);
		futureContacts[i].b->applyOffCenterForce(impulses[1], futureContacts[i].position, 1);
	}

	this->lastUpdateTime += timeStepToFirstContact;

	this->cleanUp();
}

bool sortContacts(Contact a, Contact b)
{
	return a.TOI < b.TOI;
}

std::vector<Contact> Engine::checkContacts(std::vector<Contact> contacts)
{
	if(contacts.size() > 1)
	{
		RigidBody* a = contacts[0].a;
		RigidBody* b = contacts[0].b;

		for(int i = 1; i < contacts.size(); ++i)
			if(contacts[i].a != a)
			{
				contacts[i].a = a;
				contacts[i].b = b;
				contacts[i].normal = contacts[i].normal.negate();
			}
	}

	return contacts;
}

std::vector<Contact> Engine::predictContacts()
{
	std::vector<Contact> futureContacts;

	for(int i = 1; i < this->bodies_p.size(); ++i)
		for(int j = 0; j < i; ++j)
		{
		  RigidBody* a = a->getType() == SPHERE ? a = new Sphere((Sphere&)*(this->bodies_p[i])) : a = new Box((Box&)*(this->bodies_p[i]));
			RigidBody* b = b->getType() == SPHERE ? b = new Sphere((Sphere&)*(this->bodies_p[j])) : b = new Box((Box&)*(this->bodies_p[j]));

			// Integrate forward in time.
			for(int k = 0; k < this->environmentalForces_p.size(); ++k)
			{
				this->environmentalForces_p[k]->apply(a, this->timeStep);
				this->environmentalForces_p[k]->apply(b, this->timeStep);
			}
			a->integrate(this->timeStep);
			b->integrate(this->timeStep);

			// Check for a collision.
			if(!a->isActive() && !b->isActive())
				continue;
			if(a->isBoundingBoxCollidingWith(b))
			{
				std::vector<Contact> c = a->isCollidingWith(b, this->timeStep);
				if(c.size() > 0) std::cout << "FTOI " << c[0].TOI << std::endl;

				// Set pointers to the original bodies, not their copies.
				for(int k = 0; k < c.size(); ++k)
				{
					c[k].a = this->bodies_p[i];
					c[k].b = this->bodies_p[j];
				}
				futureContacts.insert(futureContacts.end(), c.begin(), c.end());
			}

			delete a;
			delete b;
		}

	// Sort future contacts with respect to their time.
	sort(futureContacts.begin(), futureContacts.end(), sortContacts);

	return futureContacts;
}

void Engine::applyConstraints(double dt)
{
	for(int i = 0; i < this->constraints_p.size(); ++i)
		this->constraints_p[i]->apply(dt);
}

Vector3* Engine::computeImpulse(Contact contact)
{
	RigidBody* a = contact.a;
	RigidBody* b = contact.b;
  Vector3 p = contact.position;
  Vector3 n = contact.normal;

  double relativeVelocity = n * (a->getVelocity(p) - b->getVelocity(p));
	if(relativeVelocity > 0) relativeVelocity *= -1;

	std::cout << relativeVelocity << " " << p << " " << n << std::endl;

	if(relativeVelocity > -0.5)
	{
		a->setCouldSleep(true);
		b->setCouldSleep(true);
	}

	// displacements of the contact points with respect to the center of mass of each body
	Vector3 da = p - a->getPosition();
	Vector3 db = p - b->getPosition();
	
	double restitution;
	if(relativeVelocity < 0.5)
		restitution = a->getRestitution() < b->getRestitution() ? a->getRestitution() : b->getRestitution();
	else
		restitution = 0;

	Matrix3 inverseInertiaA = a->getOrientation() * a->getInverseInertiaTensor() * a->getOrientation().transpose();
	Matrix3 inverseInertiaB = b->getOrientation() * b->getInverseInertiaTensor() * b->getOrientation().transpose();
	
	double t1 = a->getInverseMass() + b->getInverseMass();
	double t2 = n * ((inverseInertiaA * (da ^ n)) ^ da);
	double t3 = n * ((inverseInertiaB * (db ^ n)) ^ db);
	
	Vector3 impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t3) * n;

	Vector3 impulseA = impulse;
	Vector3 impulseB = impulse * -1;

	std::cout << Geometry::gjkDistance(a, b) << " hinhiii" << std::endl;

	std::cout << "impulse " << impulseA << " at " << contact.position << " " << a->getPosition() - contact.position << std::endl;

	return (Vector3[]){impulseA, impulseB};
}

void Engine::applyEnvironmentalForces(RigidBody* rb_p, double dt)
{
	for(int i = 0; i < this->environmentalForces_p.size(); ++i)
		this->environmentalForces_p[i]->apply(rb_p, dt);
}

double Engine::getAbsoluteTime()
{
  struct timeval t;
  gettimeofday(&t, NULL);

  return t.tv_sec + (double)t.tv_usec / 1000000;
}

double Engine::getLocalTime()
{
  return this->getAbsoluteTime() - this->startingTime;
}

double Engine::getTimeStep()
{
  return this->timeStep;
}

void Engine::reverseTime()
{
  this->timeStep = -this->timeStep;

  for(int i = 0; i < this->bodies_p.size(); ++i)
		this->bodies_p[i]->reverseTime();
}

void Engine::cleanUp()
{
	Vector3 origin(0, 0, 0);

	for(int i = 0; i < this->bodies_p.size(); ++i)
		if((this->bodies_p[i]->getPosition() - origin).length() > 100)
		{
			this->bodies_p.erase(this->bodies_p.begin() + i);
			--i;
		}
}

void Engine::addRigidBody_p(RigidBody* rb_p)
{
  this->bodies_p.push_back(rb_p);
}

RigidBody* Engine::getBody_p(int i)
{
  return this->bodies_p[i];
}

int Engine::getBodyCount()
{
  return this->bodies_p.size();
}

void Engine::addEnvironmentalForce_p(Force* force_p)
{
  this->environmentalForces_p.push_back(force_p);
}

void Engine::addConstraint_p(Constraint* constraint_p)
{
  this->constraints_p.push_back(constraint_p);
}
