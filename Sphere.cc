#include "Sphere.h"

#include <GL/glut.h>

#include "CustomRigidBody.h"
#include "Cylinder.h"
#include "Display.h"
#include "Engine.h"

extern Engine* E;

Sphere::Sphere(double radius, double mass) :
  radius(radius)
{
	this->type = SPHERE;

	this->inverseMass= 1/mass;

	this->restitution = 0.2;
 }

 Sphere::~Sphere()
 {
 }

 void Sphere::prepare()
 {
	 if(this->fixed)
	 {
		 this->inverseMass = 0;
		 this->inverseInertiaTensor.reset();
	 }
	 else
		 this->computeInverseInertiaTensor();

	 this->computeBoundingBox();
 }

 /**
	* Compute the inverse inertia tensor
	*/
 void Sphere::computeInverseInertiaTensor()
 {
	 double i = (2.0 / 5) * (1 / this->inverseMass) * this->radius * this->radius;

	 Matrix3 inertiaTensor;
	 inertiaTensor.set(0, 0, i);
	 inertiaTensor.set(1, 1, i);
	 inertiaTensor.set(2, 2, i);

	 this->inverseInertiaTensor = inertiaTensor.inverse();
 }

 /**
	* Compute the bounding box
	*/
 void Sphere::computeBoundingBox()
 {
	 this->boundingBox.a = this->position + Vector3(-this->radius, -this->radius, -this->radius);
	 this->boundingBox.b = this->position + Vector3(this->radius, this->radius, this->radius);
 }

 void Sphere::integrate(double dt)
 {
	 RigidBody::integrate(dt);

	 this->computeBoundingBox();
 }

 void Sphere::draw()
 {
	 glPushMatrix();

	 glTranslatef(this->position.X(), this->position.Y(), this->position.Z());

	 glColor3f(this->color[0], this->color[1], this->color[2]);

	 // From http://techiethings.blogspot.com/2008/12/opengl-solid-sphere-without-glut.html
	 GLUquadricObj* quadric = gluNewQuadric();
	 gluQuadricDrawStyle(quadric, GLU_FILL);
	 gluSphere(quadric, this->radius, 20, 20);
	 gluDeleteQuadric(quadric);

  // draw the bounding box
  if(E->areBoundingBoxesDrawn())
    glutWireCube(this->radius * 2);

  glPopMatrix();
}

std::vector<Contact> Sphere::getContacts(RigidBody* rb_p)
{
	if(rb_p->getType() == SPHERE)
		return this->getContacts((Sphere*)rb_p);

	return this->getContacts((CustomRigidBody*)rb_p);
}

std::vector<Contact> Sphere::getContacts(Sphere* s_p)
{
	std::vector<Contact> contacts;

	Vector3 from1to2 = s_p->getPosition() - this->position;
	Vector3 closest1 = this->position + from1to2.normalize() * this->radius;
	Vector3 closest2 = s_p->getPosition() + (-1 * from1to2).normalize() * s_p->getRadius();

	Vector3 distance = Geometry::gjkDistance(this, s_p);

	if(distance.length() < 0.1)
	{
		Contact contact;

		contact.a = this;
		contact.b = s_p;
		contact.position = closest1 + (closest2 - closest1) / 2;
		contact.normal = distance.normalize();

		contacts.push_back(contact);

		// No need to continue, only possible contact.
		return contacts;
	}

	return contacts;
}

std::vector<Contact> Sphere::getContacts(CustomRigidBody* rb_p)
{
	std::vector<Contact> contacts;

	Vector3 distance = Geometry::gjkDistance(this, rb_p).normalize();
	Vector3 closestSphere = this->position + this->radius * distance.normalize();

  for(int i = 0; i < rb_p->structure.polygons.size(); ++i)
	{
		Polygon face = rb_p->structure.polygons[i].getPolygon();

		Vector3 distance = Geometry::gjkDistance(this, rb_p);

		if(distance.length() < 0.5)
		{
			Contact contact;

			contact.a = this;
			contact.b = rb_p;
			contact.position = closestSphere + distance;
			contact.normal = Geometry::gjkDistance(this, rb_p).normalize();

			contacts.push_back(contact);  

			// No need to continue, only one possible contact.
			return contacts;
		}
	}

  return contacts;
}

Vector3 Sphere::getSupportPoint(Vector3 direction)
{
	return this->position + direction.normalize() * this->radius;
}
