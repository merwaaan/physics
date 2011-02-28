#include "Engine.h"

#include <sys/time.h>

Engine::Engine(int* argc, char** argv, double timestep) :
  timestep(timestep),
  display(argc, argv, 400, 400, this)
{
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
  // check for collisions
  for(int i = 1; i < this->bodies_p.size(); ++i)
    for(int j = 0; j < i; ++j)
      if(this->bodies_p[i]->isBoundingBoxCollidingWith(this->bodies_p[j]))
        if(this->bodies_p[i]->isCollidingWith(this->bodies_p[j]))
          std::cout << " COLLISION BETWEEN " << i << " " << j << std::endl;

  // integrate the rigid bodies states
  for(int i = 0; i < this->bodies_p.size(); ++i)
  {
    // apply the external forces
    for(int j = 0; j < this->forces_p.size(); ++j)
      this->forces_p[j]->apply(this->bodies_p[i]);

    this->bodies_p[i]->integrate(this->timestep);
  }

  this->simulationTime += this->timestep;
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

