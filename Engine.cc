#include "Engine.h"

#include <stdlib.h>
#include <sys/time.h>

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
    std::cout << "Simulation time = " << this->simulationTime << "s" << std::endl;

    std::cout << std::endl << "COLLISION" << std::endl;

    // check for collisions
    for(int i = 1; i < this->bodies_p.size(); ++i)
      for(int j = 0; j < i; ++j)
      {
        // broad-phase test
        if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
        {
          std::cout << "Bounding box collision between " << i << " and " << j << std::endl;

          // accurate test
          Contact* contact_p = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);
        
          if(contact_p != NULL)
          {
            std::cout << "Real collision between " << i << " and " << j << std::endl;

            Vector3 impulse = this->computeImpulse(*contact_p);

            this->bodies_p[i]->applyCenterForce(-1 * impulse, 1);
            this->bodies_p[j]->applyCenterForce(impulse, 1);

            delete contact_p;
          }
        }
      }

    std::cout << std::endl << "INTEGRATION" << std::endl;

    // integrate the rigid bodies states
    for(int i = 0; i < this->bodies_p.size(); ++i)
    {      
      // apply the external forces
      for(int j = 0; j < this->forces_p.size(); ++j)
        this->forces_p[j]->apply(this->bodies_p[i], this->timeStep);

      // integrate ach body state
      this->bodies_p[i]->integrate(this->timeStep);

      std::cout << "Body #" << i << std::endl << *bodies_p[i] << std::endl;
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
  std::cout << "contact point " << p << std::endl;
  std::cout << "normal to the contact " << n << std::endl;

  double relativeVelocity = n * (contact.a->getVelocity() - contact.b->getVelocity());

  // displacements of the contact point with respect to the center of mass of each body
  Vector3 da = p - contact.a->position;
  Vector3 db = p - contact.b->position;

  double t1 = contact.a->inverseMass + contact.b->inverseMass;
  double t2 = n * ((contact.a->inverseInertiaTensor * (da ^ n)) ^ da);
  double t3 = n * ((contact.b->inverseInertiaTensor * (db ^ n)) ^ db);

  double restitution = this->timeStep > 0 ? 0.8 : 1.25;
  double impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t2);

  std::cout << "impulse " << impulse << std::endl;
  
  return impulse * contact.normal;
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

