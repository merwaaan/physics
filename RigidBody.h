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

enum Type{SPHERE, CUSTOM};

enum Dir{FORWARD, BACKWARD};

class RigidBody
{
protected:

	Type type;
	
	double inverseMass;
	Matrix3 inverseInertiaTensor;
	
	// LINEAR
	Vector3 position;
	Vector3 linearMomentum;
	Vector3 accumulatedForces;
	
	// ANGULAR
	Matrix3 orientation;
	Vector3 angularMomentum;
	Vector3 accumulatedTorques;
	
	BoundingBox boundingBox;
	
	// Cached auxiliary quantities
	Vector3 linearVelocity;
	Vector3 angularVelocity;

	double restitution;

	bool fixed;
	bool sleeping;

	int kineticEnergyLowFor;
	bool couldSleep;

public:
	RigidBody();
	~RigidBody();

	friend std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

	virtual void prepare() = 0;
	virtual void computeInverseInertiaTensor() = 0;
	virtual void computeBoundingBox() = 0;

	void clearAccumulators();
	void applyCenterForce(Vector3 force, double dt);
	void applyOffCenterForce(Vector3 force, Vector3 poa, double dt);

	virtual void integrate(double dt);
	void integrateBackward(double dt);
	void reverseTime();
	void handleSleep();

	virtual void draw() = 0;

	bool isBoundingBoxCollidingWith(RigidBody* rb_p);
	BoundingBox getBoundingBox();
    
	std::vector<Contact> isCollidingWith(RigidBody* rb_p, double dt);
	std::vector<Contact> resolveInterPenetration(RigidBody* rb_p, double dt, Dir direction, double TOI);
	virtual std::vector<Contact> getContacts(RigidBody* rb_p) = 0;
	virtual Vector3 getSupportPoint(Vector3 direction) = 0;

	double getInverseMass() { return this->isActive() ? this->inverseMass : 0; }

	Matrix3 getInverseInertiaTensor() { return this->inverseInertiaTensor; }

	void move(Vector3 displacement) { this->position += displacement; }
	void setPosition(Vector3 position) { this->position = position; }
	void setPosition(double x, double y, double z) { this->position = Vector3(x, y, z); }
	Vector3 getPosition() { return this->position; }

	void setOrientation(Matrix3 orientation) { this->orientation = orientation; }
	Matrix3 getOrientation() { return this->orientation; }

	Vector3 getVelocity() const;
	Vector3 getVelocity(const Vector3& point) const;
	double getKineticEnergy();

	void setLinearMomentum(Vector3 momentum) { this->linearMomentum = momentum; }
	void setLinearMomentum(double mx, double my, double mz) { this->linearMomentum = Vector3(mx, my, mz); }
	Vector3 getLinearMomentum() { return this->linearMomentum; }

	void setAngularMomentum(Vector3 momentum) { this->angularMomentum = momentum; }
	void setAngularMomentum(double mx, double my, double mz) { this->angularMomentum = Vector3(mx, my, mz); }
	Vector3 getAngularMomentum() { return this->angularMomentum; }

	void setFixed(bool fixed) { this->fixed = fixed; }
	bool isFixed() { return this->fixed; }

	void setSleeping(bool sleeping) { this->sleeping = sleeping; }
	bool isSleeping() { return this->sleeping; }
	bool setCouldSleep(bool couldSleep) { this->couldSleep = couldSleep; }
	bool getCouldSleep() { return this->couldSleep; }

	bool isActive() { return !this->fixed && !this->sleeping; }

	void setRestitution(double restitution) { this->restitution = restitution; }
	double getRestitution() { return this->restitution; }

	Type getType() { return this->type; }
};

#endif

