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
    std::vector<Force*> forces_p;

    double timeMultiplier;

    double lastUpdateTime;
    Display display;

  public:
    Engine(int* argc, char** argv, double timeMultiplier);
    ~Engine();

    void run();
    void update();
    bool areBoundingBoxesColliding(RigidBody* rb1_p, RigidBody* rb2_p);
    
    double getTime();
    bool needUpdate();

    void addRigidBody_p(RigidBody* rb_p);
    RigidBody* getBody_p(int i);
    int getBodyCount();

    void addForce_p(Force* force_p);
};

#endif

