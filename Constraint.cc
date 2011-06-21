#include "Constraint.h"

#include <GL/glut.h>

#include "RigidBody.h"
#include "Vector3.h"

Constraint::Constraint(RigidBody* a_p, RigidBody* b_p)
{
	this->a_p = a_p;
	this->b_p = b_p;
}

Constraint::~Constraint()
{
}

DistanceConstraint::DistanceConstraint(RigidBody* a_p, RigidBody* b_p, double distance) :
	Constraint(a_p, b_p),
	distance(distance)
{
}

DistanceConstraint::~DistanceConstraint()
{
}

void DistanceConstraint::apply(double dt)
{
	Vector3 axis = this->b_p->getPosition() - this->a_p->getPosition();
	Vector3 normal = axis.normalize();
	double relativeVelocity = normal * (this->b_p->getVelocity() - this->a_p->getVelocity());

	double relativeDistance = axis.length() - this->distance;
	double remove = relativeVelocity + relativeDistance / dt;

	Vector3 impulse = normal * (remove / (this->a_p->getInverseMass() + this->b_p->getInverseMass()));

	this->a_p->applyCenterForce(impulse, dt);
	this->b_p->applyCenterForce(-1 * impulse, dt);
}

void DistanceConstraint::draw()
{
  glPushMatrix();

	glColor3f(0, 0, 0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(this->a_p->getPosition().X(), this->a_p->getPosition().Y(), this->a_p->getPosition().Z());
	glVertex3d(this->b_p->getPosition().X(), this->b_p->getPosition().Y(), this->b_p->getPosition().Z());
	glEnd();

  glPopMatrix();
}
