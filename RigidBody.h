#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Vector3.h"
#include "Matrix3.h"

class RigidBody
{
  protected:
    double inverseMass;
    Matrix3 inverseInertiaTensor;
    
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

    virtual void prepare();
    virtual void computeInverseInertiaTensor();

    void clearAccumulators();
    void applyCenterForce(Vector3 force);
    void applyOffCenterForce(Vector3 force, Vector3 poa);
    virtual void integrate(double t);
    virtual void draw() = 0;

    void setPosition(Vector3 position);
    void setOrientation(Matrix3 orientation);
    void setFixed(bool fixed);
};

#endif

