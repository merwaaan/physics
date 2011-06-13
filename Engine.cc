#include "Engine.h"

#include <stdlib.h>
#include <sys/time.h>

#include "Geometry.h"

Engine* E = NULL;

Engine::Engine(int* argc, char** argv, double timeStep) :
	tolerance(0.09),
  timeStep(timeStep),
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
  // compute all centers of mass and inertia tensors before we begin the simulation
  for(int i = 0; i < this->bodies_p.size(); ++i)
    this->bodies_p[i]->prepare();

  this->display.run();
}

void Engine::update()
{
	// Check for collisions.
	for(int i = 1; i < this->bodies_p.size(); ++i)
		for(int j = 0; j < i; ++j)
		{
			//
			if(this->bodies_p[i]->isFixed() && this->bodies_p[j]->isFixed())
				continue;

			// Broad-phase test.
			if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
			{
				std::cout << "bounding box collision detected between #" << i << " and #" << j << std::endl;

				// Narrow-phase test.
				std::vector<Contact> contacts = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);

				if(contacts.size() > 0)
				{
					std::cout << "real collision detected between #" << i << " and #" << j << std::endl;
        
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
	this->applyConstraints(this->timeStep);

	for(int i = 0; i < this->bodies_p.size(); ++i)
	{      
		// Apply external forces.
		for(int j = 0; j < this->environmentalForces_p.size(); ++j)
			this->environmentalForces_p[j]->apply(this->bodies_p[i], this->timeStep);

		// Integrate each body.
		this->bodies_p[i]->integrate(this->timeStep);
	}

	this->lastUpdateTime += this->timeStep;

	this->cleanUp();
}

void Engine::applyConstraints(double dt)
{
	for(int i = 0; i < this->constraints_p.size(); ++i)
		this->constraints_p[i]->apply(1);
}

void Engine::emergencyPush(RigidBody* rb1_p, RigidBody* rb2_p, Vector3 distance)
{
	double totalInverseMass = rb1_p->getInverseMass() + rb2_p->getInverseMass();

	rb1_p->move(-1 * distance * rb1_p->getInverseMass() / totalInverseMass);
	rb2_p->move(distance * rb2_p->getInverseMass() / totalInverseMass);
}

Vector3* Engine::computeImpulse(Contact contact)
{
	RigidBody* a = contact.a;
	RigidBody* b = contact.b;
  Vector3 p = contact.position;
  Vector3 n = contact.normal;

  double relativeVelocity = n * (a->getVelocity(p) - b->getVelocity(p));
	Vector3 impulse;

	if(abs(relativeVelocity) < 0.5)
		impulse = relativeVelocity * n;
	else
	{		
		// displacements of the contact points with respect to the center of mass of each body
		Vector3 da = p - a->getPosition();
		Vector3 db = p - b->getPosition();

		Matrix3 inverseInertiaA = a->getOrientation() * a->getInverseInertiaTensor() * a->getOrientation().transpose();
		Matrix3 inverseInertiaB = b->getOrientation() * b->getInverseInertiaTensor() * b->getOrientation().transpose();

		double t1 = a->getInverseMass() + b->getInverseMass();
		double t2 = n * ((inverseInertiaA * (da ^ n)) ^ da);
		double t3 = n * ((inverseInertiaB * (db ^ n)) ^ db);
	
		double restitution = a->getRestitution() < b->getRestitution() ? a->getRestitution() : b->getRestitution();
		impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t3) * n;
	}

	Vector3 impulseA = impulse;
	Vector3 impulseB = -1 * impulse;

	std::cout << "impulse A " << impulseA << " at " << contact.position << std::endl;
	
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
		if((this->bodies_p[i]->getPosition() - origin).length() > 200)
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
