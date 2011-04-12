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
	Constraint(a_p, b_p)
{
	this->distance = distance;
}

DistanceConstraint::~DistanceConstraint()
{
}

void DistanceConstraint::apply(double dt)
{
	Vector3 axis = this->b_p->position - this->a_p->position;
	Vector3 normal = axis.normalize();
	double relativeVelocity = (this->b_p->getVelocity() - this->a_p->getVelocity()) * normal;

	double relativeDistance = axis.length() - this->distance;
	double remove = relativeVelocity + relativeDistance / dt;

	Vector3 impulse = (remove / (this->a_p->inverseMass + this->b_p->inverseMass)) * normal;

	this->a_p->applyCenterForce(impulse, dt);
	this->b_p->applyCenterForce(-1 * impulse, dt);
}

void DistanceConstraint::draw()
{
  glPushMatrix();

	glColor3f(1, 0, 0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(this->a_p->position.X(), this->a_p->position.Y(), this->a_p->position.Z());
	glVertex3d(this->b_p->position.X(), this->b_p->position.Y(), this->b_p->position.Z());
	glEnd();

  glPopMatrix();
}
