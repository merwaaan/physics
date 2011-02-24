#include "Engine.h"

#include <sys/time.h>

Engine::Engine(int* argc, char** argv, double timeMultiplier) :
  timeMultiplier(timeMultiplier),
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
  // compute all centers of mass before we begin the simulation
  for(int i = 0; i < this->bodies_p.size(); ++i)
    this->bodies_p[i]->computeCenterOfMass();

  this->lastUpdateTime = this->getTime();

  this->display.run();
}

void Engine::update()
{
  double t = this->getTime() - this->lastUpdateTime;

  int i, j;
  for(i = 0; i < this->bodies_p.size(); ++i)
  {
    // apply all the external forces
    for(j = 0; j < this->forces_p.size(); ++j)
      this->forces_p[j]->apply(this->bodies_p[i]);

    // integrate each body state
    this->bodies_p[i]->integrate(t * this->timeMultiplier);
  }

  this->lastUpdateTime += t;
}

double Engine::getTime()
{
  struct timeval t;
  gettimeofday(&t, NULL);

  return t.tv_sec + (double)t.tv_usec / 1000000;
}

bool Engine::needUpdate()
{
  return this->getTime() > this->lastUpdateTime + 0.016; // ~60FPS
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

