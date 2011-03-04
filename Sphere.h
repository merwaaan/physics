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

    void integrate(double dt);
    void draw();

    Contact* isCollidingWith(RigidBody* rb_p, double dt);

    Contact* isCollidingWith(Sphere* s_p, double dt);
    Contact* resolveInterPenetration(Sphere* s_p, double dt, double tolerance);
    
    Contact* isCollidingWith(CustomRigidBody* rb_p, double dt);
};

#endif
