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
  
  for(int i = 0; i < this->forces_p.size(); ++i)
    delete this->forces_p[i];
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
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "simulation time = " << this->simulationTime << "s" << std::endl;

    std::cout << std::endl << "--- COLLISION PHASE ---" << std::endl;

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

            for(int k = 0; k < contacts.size(); ++k)
            {
	            Vector3 impulse = this->computeImpulse(contacts[k]) * 100;

	            this->bodies_p[i]->applyOffCenterForce(-1 * impulse, 1, contacts[k].position);
	            this->bodies_p[j]->applyOffCenterForce(impulse, 1, contacts[k].position);
            }
          }
        }
      }

    std::cout << std::endl << "--- INTEGRATION PHASE ---" << std::endl;

    // integrate the rigid bodies states
    for(int i = 0; i < this->bodies_p.size(); ++i)
    {      
      // apply the external forces
      for(int j = 0; j < this->forces_p.size(); ++j)
        this->forces_p[j]->apply(this->bodies_p[i], this->timeStep);

      // integrate ach body state
      this->bodies_p[i]->integrate(this->timeStep);

      std::cout << "#" << i << std::endl << *bodies_p[i] << std::endl;
    }

    this->simulationTime += this->timeStep;
  }
  else
  {
    this->simulationTime = 0;
    this->reverseTime();
  }
}

Vector3 Engine::computeImpulse(Contact contact)
{
  Vector3 p = contact.position;
  Vector3 n = contact.normal;
  std::cout << "contact position = " << p << std::endl;
  std::cout << "contact normal = " << n << std::endl;

  double relativeVelocity = n * (contact.a->getVelocity() - contact.b->getVelocity());

  // displacements of the contact point with respect to the center of mass of each body
  Vector3 da = p - contact.a->position;
  Vector3 db = p - contact.b->position;

  double t1 = contact.a->inverseMass + contact.b->inverseMass;
  double t2 = n * ((contact.a->inverseInertiaTensor * (da ^ n)) ^ da);
  double t3 = n * ((contact.b->inverseInertiaTensor * (db ^ n)) ^ db);

  double restitution = this->timeStep > 0 ? 0.8 : 1.25;
  double impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t2);

  std::cout << "impulse = " << impulse * n << std::endl;

  return impulse * n;
}

void Engine::reverseTime()
{
  this->timeStep = -this->timeStep;

  for(int i = 0; i < this->bodies_p.size(); ++i)
  {
    this->bodies_p[i]->linearMomentum *= -1;
    this->bodies_p[i]->angularMomentum *= -1;
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

void Engine::addForce_p(Force* force_p)
{
  this->forces_p.push_back(force_p);
}

