#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Vector3.h"
#include "Matrix3.h"

class Sphere;
class CustomRigidBody;
class Contact;

struct BoundingBox
{
  Vector3 a;
  Vector3 b;
};

struct DerivativeState
{
  Vector3 deltaLinearMomentum;
  Vector3 deltaPosition;
};

class RigidBody
{
  public:
    double inverseMass;
    Matrix3 inverseInertiaTensor;

    BoundingBox boundingBox;
    
    Vector3 position;
    Vector3 linearMomentum;
    Vector3 accumulatedForces;

    Matrix3 orientation;
    Vector3 angularMomentum;
    Vector3 accumulatedTorques;

    bool fixed;

  public:
    RigidBody();
    ~RigidBody();

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

    virtual void prepare() = 0;
    virtual void computeInverseInertiaTensor() = 0;
    virtual void computeBoundingBox() = 0;

    void clearAccumulators();
    void applyCenterForce(Vector3 force);
    void applyOffCenterForce(Vector3 force, Vector3 poa);
    virtual void integrate(double dt);
    virtual void integrate2(double dt);
    DerivativeState evaluate(double dt, DerivativeState ds);

    virtual void draw() = 0;

    bool isBoundingBoxCollidingWith(RigidBody* rb_p);
    BoundingBox getBoundingBox();
    
    virtual Contact* isCollidingWith(RigidBody* rb_p, double dt) = 0;
    virtual Contact* isCollidingWith(Sphere* s_p, double dt) = 0;
    virtual Contact* isCollidingWith(CustomRigidBody* rb_p, double dt) = 0;

    void setPosition(Vector3 position);
    void setPosition(double x, double y, double z);
    
    void setOrientation(Matrix3 orientation);
    void setFixed(bool fixed);

    Vector3 getVelocity(double dt);
};

#endif

