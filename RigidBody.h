#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>

#include "Matrix3.h"
#include "Vector3.h"

class Contact;
class CustomRigidBody;
class Sphere;

struct BoundingBox
{
  Vector3 a;
  Vector3 b;
};

class RigidBody
{
  public:
    double inverseMass;
    Matrix3 inverseInertiaTensor;

    // linear component
    Vector3 position;
    Vector3 linearMomentum;
    Vector3 accumulatedForces;

    // angular component
    Matrix3 orientation;
    Vector3 angularMomentum;
    Vector3 accumulatedTorques;

    BoundingBox boundingBox;

    // cached auxiliary quantities
    Vector3 linearVelocity;
    Vector3 angularVelocity;

		double restitution;
		double friction;

    bool fixed;

  public:
    RigidBody();
    ~RigidBody();

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

    virtual void prepare() = 0;
    virtual void computeInverseInertiaTensor() = 0;
    virtual void computeBoundingBox() = 0;

    void clearAccumulators();
    void applyCenterForce(Vector3 force, double dt);
    void applyOffCenterForce(Vector3 force, double dt, Vector3 poa);
    void computeAuxiliaryQuantities();

    virtual void integrate(double dt);
		void integrateBackward(double dt);
		void reverseTime();

    virtual void draw() = 0;

    bool isBoundingBoxCollidingWith(RigidBody* rb_p);
    BoundingBox getBoundingBox();
    
    virtual std::vector<Contact> isCollidingWith(RigidBody* rb_p, double dt) = 0;
    virtual std::vector<Contact> isCollidingWith(Sphere* s_p, double dt) = 0;
    virtual std::vector<Contact> isCollidingWith(CustomRigidBody* rb_p, double dt) = 0;

    void setPosition(Vector3 position);
    void setPosition(double x, double y, double z);
    void setOrientation(Matrix3 orientation);
    void setFixed(bool fixed);

    Vector3 getPosition() const;
    Vector3 getVelocity() const;
    Vector3 getVelocity(const Vector3& point) const;

		void setRestitution(double restitution);
		double getRestitution() const;
		void setFriction(double friction);
		double getFriction() const;
};

#endif

