#ifndef FORCE_H
#define FORCE_H

#include "Vector3.h"
#include "RigidBody.h"

class Force
{
protected:
	Vector3 force;

public:
	Force(Vector3 force);
	~Force();

	virtual void apply(RigidBody* rb_p, double dt) = 0;
};

class CenterForce : public Force
{
public:
	CenterForce(Vector3 force);
	~CenterForce();

	virtual void apply(RigidBody* rb_p, double dt);
};

class OffCenterForce : public Force
{
protected:
	Vector3 pointOfApplication;
	
public:
	OffCenterForce(Vector3 force, Vector3 poa);
	~OffCenterForce();
	
	virtual void apply(RigidBody* rb_p, double dt);
};

class Gravity : public CenterForce
{
public:
	Gravity(Vector3 force);
	~Gravity();

	virtual void apply(RigidBody* rb_p, double dt);
};

#endif
