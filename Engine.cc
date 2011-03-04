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
    // check for collisions
    for(int i = 1; i < this->bodies_p.size(); ++i)
      for(int j = 0; j < i; ++j)
      {
        // bounding boxes test
        if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
        {
          std::cout << "TESTING " << i << " AND " << j << std::endl;

          // accurate test
          Contact* contact_p = this->bodies_p[i]->isCollidingWith(this->bodies_p[j], this->timeStep);
        
          if(contact_p != NULL)
          {
            Vector3 impulse = this->computeImpulse(*contact_p);

            this->bodies_p[j]->applyCenterForce(impulse);
            this->bodies_p[i]->applyCenterForce(-1 * impulse);

            delete contact_p;
          }
        }
      }

    // integrate the rigid bodies states
    for(int i = 0; i < this->bodies_p.size(); ++i)
    {      
      // apply the external forces
      for(int j = 0; j < this->forces_p.size(); ++j)
        this->forces_p[j]->apply(this->bodies_p[i]);

      this->bodies_p[i]->integrate(this->timeStep);
    
      // erase the forces from the last frame
      this->bodies_p[i]->clearAccumulators();
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

  double relativeVelocity = n * (contact.a->getVelocity() - contact.b->getVelocity());

  double t1 = contact.a->inverseMass + contact.b->inverseMass;
  double t2 = n * ((contact.a->inverseInertiaTensor * (p ^ n)) ^ p);
  double t3 = n * ((contact.b->inverseInertiaTensor * (p ^ n)) ^ p);

  double restitution = this->timeStep > 0 ? 0.8 : 1.25;
  double impulse = (-(1 + restitution) * relativeVelocity) / (t1 + t2 + t2);
  
  return impulse * contact.normal;
}

void Engine::reverseTime()
{
  this->timeStep = -this->timeStep;

  for(int i = 0; i < this->bodies_p.size(); ++i)
    this->bodies_p[i]->linearMomentum = this->bodies_p[i]->linearMomentum * -1;
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

