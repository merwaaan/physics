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

    std::vector<Contact> isCollidingWith(RigidBody* rb_p, double dt);

    std::vector<Contact> isCollidingWith(Sphere* s_p, double dt);
    std::vector<Contact> resolveInterPenetration(Sphere* s_p, double dt);
    
    std::vector<Contact> isCollidingWith(CustomRigidBody* rb_p, double dt);

    double getRadius() const;
};

#endif
