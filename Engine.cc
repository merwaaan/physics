#include "Engine.h"

#include <stdlib.h>
#include <sys/time.h>

#include "Geometry.h"

Engine* E = NULL;

Engine::Engine(int* argc, char** argv, double timeStep) :
	tolerance(0.01),
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
	//std::cout << "---------------------------------------" << std::endl;
	//std::cout << "simulation time = " << this->lastUpdateTime << "s" << std::endl;

	// std::cout << std::endl << "--- COLLISION PHASE ---" << std::endl;

	// Check for collisions.
	for(int i = 1; i < this->bodies_p.size(); ++i)
		for(int j = 0; j < i; ++j)
		{
			// Broad-phase test
			if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
			{
				std::cout << "bounding box collision detected between #" << i << " and #" << j << std::endl;

				// Narrow-phase test
				std::vector<Contact> contacts = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);
        
				if(contacts.size() > 0)
				{
					std::cout << "real collision detected between #" << i << " and #" << j << std::endl;

					for(int k = 0; k < contacts.size(); ++k)
					{
						Vector3* impulses = this->computeImpulse(contacts[k]);

						contacts[k].a->applyOffCenterForce(impulses[0], 1, contacts[k].position);
						contacts[k].b->applyOffCenterForce(impulses[1], 1, contacts[k].position);
					}
				}
			}
		}

	// Apply constraints.
	for(int i = 0; i < this->constraints_p.size(); ++i)
		this->constraints_p[i]->apply(0.5);

	//std::cout << std::endl << "--- INTEGRATION PHASE ---" << std::endl;

	for(int i = 0; i < this->bodies_p.size(); ++i)
	{      
		// Apply external forces.
		for(int j = 0; j < this->environmentalForces_p.size(); ++j)
			this->environmentalForces_p[j]->apply(this->bodies_p[i], this->timeStep);

		// Integrate each body.
		this->bodies_p[i]->integrate(this->timeStep);

		//std::cout << "#" << i << std::endl << *bodies_p[i] << std::endl;
	}

	this->lastUpdateTime += this->timeStep;

	//this->cleanUp();
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
		// displacements of the contact point with respect to the center of mass of each body
		Vector3 da = p - a->position;
		Vector3 db = p - b->position;

		Matrix3 inverseInertiaA = a->orientation * a->inverseInertiaTensor * a->orientation.transpose();
		Matrix3 inverseInertiaB = b->orientation * b->inverseInertiaTensor * b->orientation.transpose();

		double t1 = a->inverseMass + b->inverseMass;
		double t2 = n * ((inverseInertiaA * (da ^ n)) ^ da);
		double t3 = n * ((inverseInertiaB * (db ^ n)) ^ db);
	
		double restitution = this->timeStep > 0 ? 0.25 : 1.25;
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

void Engine::reverseTime()
{
  this->timeStep = -this->timeStep;

  for(int i = 0; i < this->bodies_p.size(); ++i)
		this->bodies_p[i]->reverseTime();
}

double Engine::getTimeStep()
{
  return this->timeStep;
}

void Engine::cleanUp()
{
	Vector3 origin(0, 0, 0);

	for(int i = 0; i < this->bodies_p.size(); ++i)
		if((this->bodies_p[i]->position - origin).length() > 50)
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

Constraint* Engine::getConstraint_p(int i)
{
  return this->constraints_p[i];
}

int Engine::getConstraintCount()
{
  return this->constraints_p.size();
}
