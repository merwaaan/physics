#include "Engine.h"

#include <stdlib.h>
#include <sys/time.h>

#include "Geometry.h"

Engine* engine_pg = NULL;

Engine::Engine(int* argc, char** argv, double timeStep) :
  timeStep(timeStep),
  display(argc, argv, 600, 600, this)
{
  engine_pg = this;
}

Engine::~Engine()
{
  for(int i = 0; i < this->bodies_p.size(); ++i)
    delete this->bodies_p[i];

	for(int i = 0; i < this->environmentalForces_p.size(); ++i)
		delete this->environmentalForces_p[i];
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
  if(this->simulationTime >= 0)
  {
    ///std::cout << "---------------------------------------" << std::endl;
    //std::cout << "simulation time = " << this->simulationTime << "s" << std::endl;

		// std::cout << std::endl << "--- COLLISION PHASE ---" << std::endl;

    // check for collisions
    for(int i = 1; i < this->bodies_p.size(); ++i)
      for(int j = 0; j < i; ++j)
      {
        // broad-phase test
        if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
        {
          std::cout << "bounding box collision detected between #" << i << " and #" << j << std::endl;

          // accurate test
          std::vector<Contact> contacts = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);
        
          if(contacts.size() > 0)
          {
            std::cout << "real collision detected between #" << i << " and #" << j << std::endl;
						std::cout << contacts.size() << std::endl;
            for(int k = 0; k < contacts.size(); ++k)
            {
	            Vector3* impulses = this->computeImpulse(contacts[k]);

	            contacts[k].a->applyOffCenterForce(impulses[0], 1, contacts[k].position);
	            contacts[k].b->applyOffCenterForce(impulses[1], 1, contacts[k].position);
            }
					}
        }
      }

    //std::cout << std::endl << "--- INTEGRATION PHASE ---" << std::endl;

    // integrate the rigid bodies states
    for(int i = 0; i < this->bodies_p.size(); ++i)
    {      
      // apply the external forces
      for(int j = 0; j < this->environmentalForces_p.size(); ++j)
        this->environmentalForces_p[j]->apply(this->bodies_p[i], this->timeStep);

      // integrate each body state
      this->bodies_p[i]->integrate(this->timeStep);

      if(i == 0 || i == 1) std::cout << "#" << i << std::endl << *bodies_p[i] << std::endl;
    }

    this->simulationTime += this->timeStep;

		this->cleanUp();
  }
  else
  {
    this->simulationTime = 0;
    this->reverseTime();
  }
}

Vector3* Engine::computeImpulse(Contact contact)
{
	RigidBody* a = contact.a;
	RigidBody* b = contact.b;
  Vector3 p = contact.position;
  Vector3 n = contact.normal;

  double relativeVelocity = n * (a->getVelocity(p) - b->getVelocity(p));

	/**
	 * RESTING CONTACT
	 */

/*	if(relativeVelocity < 0.5)
	{
		Vector3 impulseA = relativeVelocity * n;
		Vector3 impulseB = -1 * relativeVelocity * n;

		return (Vector3[]){impulseA, impulseB};
	}
*/
	/**
	 * SEPARATING CONTACT
	 */

  // displacements of the contact point with respect to the center of mass of each body
  Vector3 da = p - a->position;
  Vector3 db = p - b->position;

	Matrix3 inverseInertiaA = a->orientation * a->inverseInertiaTensor * a->orientation.transpose();
 	Matrix3 inverseInertiaB = b->orientation * b->inverseInertiaTensor * b->orientation.transpose();

  double t1 = a->inverseMass + b->inverseMass;
  double t2 = n * ((inverseInertiaA * (da ^ n)) ^ da);
  double t3 = n * ((inverseInertiaB * (db ^ n)) ^ db);

  double restitution = this->timeStep > 0 ? 0.8 : 1.25;
	Vector3 impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t3) * n;

	std::cout << "impulse " << impulse << " at " << contact.position << std::endl;

	Vector3 impulseA = impulse;
	Vector3 impulseB = -1 * impulse;// mass?;

	return (Vector3[]){impulseA, impulseB};
}

void Engine::applyEnvironmentalForces(RigidBody* rb_p, double dt)
{
	for(int i = 0; i < this->environmentalForces_p.size(); ++i)
		this->environmentalForces_p[i]->apply(rb_p, dt);
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
		if((this->bodies_p[i]->position - origin).length() > 20)
		{
			this->bodies_p.erase(this->bodies_p.begin() + i);
			--i;
		}
}

double Engine::getTimeStep()
{
  return this->timeStep;
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
