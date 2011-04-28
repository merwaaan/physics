#ifndef SPHERE_H
#define SPHERE_H

#include "RigidBody.h"

class Sphere : public RigidBody
{
  protected:
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

		void setRadius(double radius) { this->radius = radius; }
    double getRadius() const { return this->radius; }
};

#endif
