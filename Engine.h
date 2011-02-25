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
    double getTime();
    bool needUpdate();

    void addRigidBody_p(RigidBody* rb_p);
    RigidBody* getBody_p(int i);
    int getBodyCount();

    void addForce_p(Force* force_p);

    void setKeyboardCallback_p(void(*func)(unsigned char k, int x, int y));
};

#endif

