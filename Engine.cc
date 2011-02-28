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
  for(int i = 0; i < this->bodies_p.size(); ++i)
  {
    // apply all the external forces
    for(int j = 0; j < this->forces_p.size(); ++j)
      this->forces_p[j]->apply(this->bodies_p[i]);

    // integrate each body state
    this->bodies_p[i]->integrate(this->timestep);
  }

  // check for collisions
  for(int i = 1; i < this->bodies_p.size(); ++i)
    for(int j = 0; j < i; ++j)
      if(this->areBoundingBoxesColliding(this->bodies_p[i], this->bodies_p[j]))
        if(this->areColliding(this->bodies_p[i], this->bodies_p[j]))
          std::cout << " COLLISION BETWEEN " << i << " " << j << std::endl;

  this->simulationTime += this->timestep;
}

bool Engine::areBoundingBoxesColliding(RigidBody* a, RigidBody* b)
{
  BoundingBox b1 = a->getBoundingBox();
  BoundingBox b2 = b->getBoundingBox();

  if(
    b1.a.X() > b2.b.X() || b1.b.X() < b2.a.X() ||
    b1.a.Y() > b2.b.Y() || b1.b.Y() < b2.a.Y() ||
    b1.a.Z() > b2.b.Z() || b1.b.Z() < b2.a.Z())
    return false;

  return true;
}

bool Engine::areColliding(RigidBody* a, RigidBody* b)
{
  return false;
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

