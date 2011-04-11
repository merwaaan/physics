#include "Constraint.h"

Constraint::Constraint(RigidBody* a_p, RigidBody* b_p)
{
	this->a_p = a_p;
	this->b_p = b_p;
}

Constraint::~Constraint()
{
}

DistanceConstraint::DistanceConstraint(RigidBody* a_p, RigidBody* b_p, double distance) :
	Constraint(a_p, b_p)
{
	this->distance = distance;
}

DistanceConstraint::~DistanceConstraint()
{
}

void DistanceConstraint::apply(double dt)
{
	Vector axis = this->b_p->position - this->a_p->position;
	double relativeVelocity = (this->b_p->getVelocity() - this->a_p->getVelocity) * axis.normalize();
	double currentDistance = axis.length();

	double relativeDistance = currentDistance - this->distance;
	double remove = relativeVelocity + relativeDistance / dt;

	Vector3 impulse = (remove / (this->a_p->inverseMass + this->b_p->inverseMass)) * axis.normalize();

	this->a_p->applyCenterForce(impulse);
	this->b_p->applyCenterForce(-1 * impulse);
}
