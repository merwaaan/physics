#include "CustomRigidBody.h"

#include <GL/glut.h>
#include <limits>

#include "Display.h"
#include "Engine.h"
#include "Sphere.h"

extern Engine* E;

std::vector<Edge> Structure::getEdges() const
{
	std::vector<Edge> edges;

	// List all the edges.
	for(int i = 0; i < this->polygons.size(); ++i)
	{
		for(int j = 0; j < this->polygons[i].size - 1; ++j)
		{
			Vector3 a = this->polygons[i].vertices_p[j]->absPosition;
			Vector3 b = this->polygons[i].vertices_p[j + 1]->absPosition;
			edges.push_back((Edge){a, b});
		}

		// Connect the last and the first vertex.
		Vector3 a = this->polygons[i].vertices_p[this->polygons[i].size - 1]->absPosition;
		Vector3 b = this->polygons[i].vertices_p[0]->absPosition;
		edges.push_back((Edge){a, b});
	}
 
	// Remove redundant edges.
	for(int i = 0; i < edges.size(); ++i)
		for(int j = i + 1; j < edges.size(); ++j)
			if(edges[i].a == edges[j].a && edges[i].b == edges[j].b ||
			   edges[i].a == edges[j].b && edges[i].b == edges[j].a)
				edges.erase(edges.begin() + j--);

	return edges;
}

CustomRigidBody::CustomRigidBody()
{
}

CustomRigidBody::~CustomRigidBody()
{
  for(int i = 0; i < this->structure.polygons.size(); ++i)
    delete[] this->structure.polygons[i].vertices_p;
}

/**
 * Add a vertex to the structure of the rigid body
 */
void CustomRigidBody::addVertex(int id, double x, double y, double z, double m)
{
  CustomVertex v;
  v.id = id;
  v.localPosition = Vector3(x, y, z);
  v.mass = m;

  this->structure.vertices.push_back(v);
}

/**
 * Add a polygon to the structure of the rigid body
 */
void CustomRigidBody::addPolygon(int count, int* ids)
{
  CustomPolygon p;
  p.size = count;
  p.vertices_p = new CustomVertex*[count];

  // link all referenced vertices with a polygon
  for(int i = 0; i < count; ++i)
    p.vertices_p[i] = getVertexById_p(ids[i]);

  this->structure.polygons.push_back(p);
}

/**
 * Prepare the rigid body for the simulation by precomputing some variables
 */
void CustomRigidBody::prepare()
{
  if(this->fixed)
  {
    this->inverseMass = 0;
    this->inverseInertiaTensor.reset();
  }
  else
  {
    this->computeCenterOfMass();
    this->computeInverseInertiaTensor();
  }

  this->computeVerticesAbsolutePositions();
  this->computeBoundingBox();
}

/**
 * Compute the position of the center of mass with respect to the vertices positions and
 * to their mass
 */
void CustomRigidBody::computeCenterOfMass()
{
  double totalMass = 0;
  Vector3 centerOfMass;

  for(int i = 0; i < this->structure.vertices.size(); ++i)
  {
    CustomVertex* v_p = &this->structure.vertices[i];

    centerOfMass += v_p->localPosition * v_p->mass;  
    totalMass += v_p->mass;
  }

  this->inverseMass = 1 / totalMass;
  this->position += centerOfMass * this->inverseMass;
}

void CustomRigidBody::computeInverseInertiaTensor()
{
  // TODO
}

/**
 * Compute the axis-aligned bounding box
 */
void CustomRigidBody::computeBoundingBox()
{
  double min[3], max[3];

  min[0] = min[1] = min[2] = std::numeric_limits<double>::max();
  max[0] = max[1] = max[2] = -min[0];

  for(int i = 0; i < this->structure.vertices.size(); ++i)
  {
    Vector3 p = this->structure.vertices[i].absPosition;

		min[0] = p.X() < min[0] ? p.X() : min[0];
		min[1] = p.Y() < min[1] ? p.Y() : min[1];
		min[2] = p.Z() < min[2] ? p.Z() : min[2];

		max[0] = p.X() > max[0] ? p.X() : max[0];
		max[1] = p.Y() > max[1] ? p.Y() : max[1];
		max[2] = p.Z() > max[2] ? p.Z() : max[2];
	}

  this->boundingBox.a = Vector3(min[0], min[1], min[2]);
  this->boundingBox.b = Vector3(max[0], max[1], max[2]);
}

/**
 * Integrate the state of the rigid body
 */
void CustomRigidBody::integrate(double dt)
{
  RigidBody::integrate(dt);

  this->computeVerticesAbsolutePositions();
  this->computeBoundingBox();
}

/**
 * Draw the body
 */
void CustomRigidBody::draw()
{
  // draw each polygon
  for(int i = 0; i < this->structure.polygons.size(); ++i)
  {
    Vector3 normal = this->structure.polygons[i].getNormal();

    glBegin(GL_POLYGON);

    // draw each vertex
    for(int j = 0; j < this->structure.polygons[i].size; ++j)
    {
      glNormal3f(normal.X(), normal.Y(), normal.Z());

      glVertex3d(
        this->structure.polygons[i].vertices_p[j]->absPosition.X(),
        this->structure.polygons[i].vertices_p[j]->absPosition.Y(),
        this->structure.polygons[i].vertices_p[j]->absPosition.Z());
    }

    glEnd();
  }

  // draw the bounding box
  if(E->areBoundingBoxesDrawn())
  {
    glPushMatrix();

    Vector3 center = (this->boundingBox.a + this->boundingBox.b) * 0.5;
    glTranslatef(center.X(), center.Y(), center.Z());
    
    glScalef(
        this->boundingBox.a.X() - this->boundingBox.b.X(),
        this->boundingBox.a.Y() - this->boundingBox.b.Y(),
        this->boundingBox.a.Z() - this->boundingBox.b.Z());

    glutWireCube(1);
    
    glPopMatrix();
  }
}

/**
 * Compute the vertices absolute positions with respect to their local positions and to
 * the position of the center of mass and the orientation of the body
 */
void CustomRigidBody::computeVerticesAbsolutePositions()
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
    this->structure.vertices[i].absPosition = this->orientation * this->structure.vertices[i].localPosition + this->position;
}

/**
 * Double-dispatch
 */
std::vector<Contact> CustomRigidBody::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

/**
 * Check for a collision with a sphere
 */
std::vector<Contact> CustomRigidBody::isCollidingWith(Sphere* s_p, double dt)
{
	return std::vector<Contact>();
}

/**
 * Check for a collision with a custom rigid body
 */
std::vector<Contact> CustomRigidBody::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  // If at least one separating plane exists, there is no collision.
	if(Geometry::findSeparatingPlane(this, rb_p))
  {
	  std::cout << "separating plane found : no collision" << std::endl;

    return std::vector<Contact>();
  }

  std::cout << "no separating plane found : collision" << std::endl;

	double sdt = -dt / 10;
	int backtrackings = 0;

	this->reverseTime();
	rb_p->reverseTime();

	// Separate the bodies to help speed up the penetration resolution.
	while(!Geometry::findSeparatingPlane(this, rb_p) && backtrackings < 10)
	{
    std::cout << "going backwarddddd " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

		this->integrate(sdt);
    rb_p->integrate(sdt);

		++backtrackings;
	}

	// Apply an emergency push if the bodies are stuck.
	if(!Geometry::findSeparatingPlane(this, rb_p))
		E->emergencyPush(this, rb_p);

	this->reverseTime();
	rb_p->reverseTime();

	std::cout << "NEXT STEP" << std::endl;
  return this->resolveInterPenetration(rb_p, sdt * backtrackings);
}

/**
 * Determine the exact contact point by integrating backward in time
 */
std::vector<Contact> CustomRigidBody::resolveInterPenetration(CustomRigidBody* rb_p, double dt)
{
  // compute the distance between the two bodies
	bool interPenetration;
  Vector3 distance = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &interPenetration);

  std::cout << "d = " << distance.length() << " ip = " << interPenetration << std::endl;

  // if the bodies are too far apart, integrate forward in time
  if(!interPenetration && distance.length() > E->getTolerance())
  {
		double sdt = dt / 2;

	  std::cout << "going forward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrate(sdt);
    rb_p->integrate(sdt);

    std::cout << *rb_p << std::endl;

    return this->resolveInterPenetration(rb_p, sdt);
  }
  // else if the bodies are inter-penetrating, integrate backward in time
  else if(interPenetration)
  {
		double sdt = dt / 2;

	  std::cout << "going backward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

    this->integrateBackward(sdt);
    rb_p->integrateBackward(sdt);

		std::cout << *rb_p << std::endl;

    return this->resolveInterPenetration(rb_p, sdt);
  }

  // If bodies are within the tolerance area, compute the real contact points.
	std::vector<Contact> vfContacts = Geometry::vertexFaceContacts(this, rb_p);
	std::vector<Contact> eeContacts = Geometry::edgeEdgeContacts(this, rb_p);
	std::cout << vfContacts.size() << " v/f and " << eeContacts.size() << " e/e" << std::endl;

	for(int i = 0; i < eeContacts.size(); ++i)
		vfContacts.push_back(eeContacts[i]);

	// recompute auxiliary quantities as they could have been
	// corrupted during the binary search
	vfContacts[0].a->computeAuxiliaryQuantities();
	vfContacts[0].b->computeAuxiliaryQuantities();

	return vfContacts;
}

Vector3 CustomRigidBody::getSupportPoint(Vector3 direction)
{
	int indexBest = 0;
  double bestLength = this->structure.vertices[0].absPosition * direction;

  for(int i = 1; i < this->structure.vertices.size(); ++i)
  {
    double length = this->structure.vertices[i].absPosition * direction;

    if(length > bestLength)
    {
      indexBest = i;
      bestLength = length;
    }
  }

  return this->structure.vertices[indexBest].absPosition;
}

/**
 * Return the vertex with the given key
 */
CustomVertex* CustomRigidBody::getVertexById_p(int id)
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
    if(this->structure.vertices[i].id == id)
      return &(this->structure.vertices[i]);

  return NULL;
}

