#ifndef ENGINE_H
#define ENGINE_H

#include <vector>

#include "Display.h"
#include "RigidBody.h"
#include "Force.h"

class Engine
{
  private:
    std::vector<RigidBody*> bodies_p;
    std::vector<Force*> environmentalForces_p;

    double timeStep;
    double simulationTime;

    Display display;

  public:
    Engine(int* argc, char** argv, double timestep);
    ~Engine();

    void run();
    void update();
    Vector3* computeImpulse(Contact contact);
		void applyEnvironmentalForces(RigidBody* rb_p, double dt);
		void cleanUp();

    void reverseTime();
    double getTimeStep();

    void addRigidBody_p(RigidBody* rb_p);
    RigidBody* getBody_p(int i);
    int getBodyCount();

    void addEnvironmentalForce_p(Force* force_p);
};

#endif

