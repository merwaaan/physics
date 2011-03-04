#ifndef ENGINE_H
#define ENGINE_H

#include <vector>

#include "Display.h"
#include "RigidBody.h"
#include "Force.h"

struct Contact
{
  RigidBody* a;
  RigidBody* b;

  Vector3 position;
  Vector3 normal;
};

class Engine
{
  private:
    std::vector<RigidBody*> bodies_p;
    std::vector<Force*> forces_p;

    double timeStep;
    double simulationTime;

    Display display;

  public:
    Engine(int* argc, char** argv, double timestep);
    ~Engine();

    void run();
    void update();
    Vector3 computeImpulse(Contact contact);

    void reverseTime();

    void addRigidBody_p(RigidBody* rb_p);
    RigidBody* getBody_p(int i);
    int getBodyCount();

    void addForce_p(Force* force_p);
};

#endif

