#include "CustomRigidBody.h"

#include <GL/glut.h>
#include <limits>

#include "Display.h"
#include "Engine.h"
#include "Sphere.h"

extern Engine* E;

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
	this->computeCachedEdges();
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

void CustomRigidBody::computeCachedEdges()
{
	std::vector<CachedEdge> cache;

	// List all the edges.
	for(int i = 0; i < this->structure.polygons.size(); ++i)
	{
		for(int j = 0; j < this->structure.polygons[i].size - 1; ++j)
		{
			int idA = this->structure.polygons[i].vertices_p[j]->id;
			int idB = this->structure.polygons[i].vertices_p[j + 1]->id;
			cache.push_back((CachedEdge){idA, idB});
		}

		// Connect the last and the first vertex.
		int idA = this->structure.polygons[i].vertices_p[this->structure.polygons[i].size - 1]->id;
		int idB = this->structure.polygons[i].vertices_p[0]->id;
		cache.push_back((CachedEdge){idA, idB});
	}
 
	// Remove redundant edges.
	for(int i = 0; i < cache.size(); ++i)
		for(int j = i + 1; j < cache.size(); ++j)
			if(cache[i].idA == cache[j].idA && cache[i].idA == cache[j].idB ||
			   cache[i].idA == cache[j].idB && cache[i].idB == cache[j].idA)
				cache.erase(cache.begin() + j--);

	this->cachedEdges = cache;
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
	bool ip;
	Vector3 dist = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &ip);
	std::cout << "ip " << ip << " dist " << dist << std::endl;

  // Check for a true collision.
	if(!ip)
  {
	  std::cout << "Narrow-phase detection : NO " << std::endl;

    return std::vector<Contact>();
  }
 
	std::cout << "Narrow-phase detection : YES " << std::endl;

	int sdiv = 5;
	double sdt = -dt / sdiv;

	this->reverseTime();
	rb_p->reverseTime();

	// Separate the bodies to speed up the penetration resolution.
	int backtracks = 0;
	for(int i = 0; i < sdiv; ++i)
	{
    std::cout << "going backward " << sdt << "ms" << std::endl;

	  E->applyEnvironmentalForces(this, sdt);
	  E->applyEnvironmentalForces(rb_p, sdt);

		this->integrate(sdt);
		rb_p->integrate(sdt);

		++backtracks;
		std::cout << rb_p->position << std::endl;

    // Exit the loop when no more interpenetration.
		Vector3 dist = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p, &ip);
		std::cout << "ip " << ip << " dist " << dist << std::endl;
		if(!ip)
			break;
	}

	this->reverseTime();
	rb_p->reverseTime();

	// Apply an emergency push if the bodies are stuck.
/*	if(!Geometry::findSeparatingPlane(this, rb_p))
	{
		std::cout << "push!!!" << std::endl;
		E->emergencyPush(this, rb_p);
	}
*/
	std::cout << "NEXT STEP" << std::endl;
  return this->resolveInterPenetration(rb_p, sdt * backtracks);
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
	if(vfContacts.size() > 0)
	{
		vfContacts[0].a->computeAuxiliaryQuantities();
		vfContacts[0].b->computeAuxiliaryQuantities();
	}

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

std::vector<Edge> CustomRigidBody::getEdges()
{
	std::vector<Edge> edges;

	for(int i = 0; i < this->cachedEdges.size(); ++i)
	{
		Vector3 posA = this->structure.vertices[this->cachedEdges[i].idA].absPosition;
		Vector3 posB = this->structure.vertices[this->cachedEdges[i].idB].absPosition;
		edges.push_back((Edge){posA, posB});
	}

	return edges;
}
