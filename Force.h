#ifndef FORCE_H
#define FORCE_H

#include "Vector3.h"
#include "RigidBody.h"

class Force
{
  private:
    bool on;

  public:
    Force();
    ~Force();

    virtual void apply(RigidBody* rb_p) = 0;
};

class Gravity : public Force
{
  private:
    Vector3 force;

  public:
    Gravity(Vector3 force);
    ~Gravity();

    virtual void apply(RigidBody* rb_p);
};

class AirFriction : public Force
{
  private:


  public:
    AirFriction();
    ~AirFriction();
};

#endif

