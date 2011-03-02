#ifndef SPHERE_H
#define SPHERE_H

#include "RigidBody.h"

class Sphere : public RigidBody
{
  private:
    double radius;

  public:
    Sphere(double radius);
    ~Sphere();

    void prepare();
    void computeInverseInertiaTensor();
    void computeBoundingBox();

    void integrate(double t);
    void draw();

    Contact* isCollidingWith(RigidBody* rb_p);
    Contact* isCollidingWith(Sphere* s_p);
    Contact* isCollidingWith(CustomRigidBody* rb_p);
};

#endif
