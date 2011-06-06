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

		std::vector<Contact> getContacts(RigidBody* rb_p);
		std::vector<Contact> getContacts(Sphere* s_p);
		std::vector<Contact> getContacts(CustomRigidBody* s_p);
    Vector3 getSupportPoint(Vector3 direction);

		void setRadius(double radius) { this->radius = radius; }
    double getRadius() const { return this->radius; }
};

#endif
