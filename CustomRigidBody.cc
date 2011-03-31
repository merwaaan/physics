#include "CustomRigidBody.h"

#include <GL/glut.h>
#include <limits>

#include "Display.h"
#include "Engine.h"
#include "Sphere.h"

extern Display* display_pg;
extern Engine* engine_pg;

std::vector<Edge> Structure::getEdges() const
{
	std::vector<Edge> edges;

	// list all the edges
	for(int i = 0; i < this->polygons.size(); ++i)
	{
		for(int j = 0; j < this->polygons[i].size - 1; ++j)
		{
			Vector3 v1 = this->polygons[i].vertices_p[j]->absPosition;
			Vector3 v2 = this->polygons[i].vertices_p[j + 1]->absPosition;
			edges.push_back((Edge){v1, v2});
		}

		// connect the last and the first vertex
		Vector3 v1 = this->polygons[i].vertices_p[this->polygons[i].size - 1]->absPosition;
		Vector3 v2 = this->polygons[i].vertices_p[0]->absPosition;
		edges.push_back((Edge){v1, v2});
	}
 
	// remove redundant edges
	for(int i = 0; i < edges.size(); ++i)
		for(int j = i + 1; j < edges.size(); ++j)
			if(edges[i].a == edges[j].a && edges[i].b == edges[j].b ||
			   edges[i].a == edges[j].b && edges[i].b == edges[j].a)
			{
				edges.erase(edges.begin() + j);
				--j;
			}

	return edges;
}

/**
 * Return the best support point along a given direction
 */
Vector3 Structure::getSupportPoint(Vector3 direction) const
{
  int bestIndex = 0;
  double bestLength = this->vertices[0].absPosition * direction;

  for(int i = 1; i < this->vertices.size(); ++i)
    if(this->vertices[i].absPosition * direction > bestLength)
    {
      bestIndex = i;
      bestLength = this->vertices[i].absPosition * direction;
    }

  return this->vertices[bestIndex].absPosition;
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
  double minx, miny, minz, maxx, maxy, maxz;

  minx = miny = minz = std::numeric_limits<double>::max();
  maxx = maxy = maxz = -minx;

  for(int i = 0; i < this->structure.vertices.size(); ++i)
  {
    Vector3 pos = this->structure.vertices[i].absPosition;

    if(pos.X() < minx)
      minx = pos.X();
    if(pos.X() > maxx)
      maxx = pos.X();
    
    if(pos.Y() < miny)
      miny = pos.Y();
    if(pos.Y() > maxy)
      maxy = pos.Y();

    if(pos.Z() < minz)
      minz = pos.Z();
    if(pos.Z() > maxz)
      maxz = pos.Z();
  }

  this->boundingBox.a = Vector3(minx, miny, minz);
  this->boundingBox.b = Vector3(maxx, maxy, maxz);
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
  if(display_pg->areBoundingBoxesDrawn())
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
}

/**
 * Check for a collision with a custom rigid body
 */
std::vector<Contact> CustomRigidBody::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  double tolerance = 0.2;

  // if at least one separation plane exists, there is no collision
  if(this->findSeparationPlane(rb_p) || rb_p->findSeparationPlane(this))
  {
	  std::cout << "separation plane found : no collision" << std::endl;
    std::vector<Contact> contacts;

    return contacts;
  }

  std::cout << "no separation plane found : collision!" << std::endl;
  return this->resolveInterPenetration(rb_p, dt, tolerance);
}

/**
 * Return true if a separation plane exists between the two bodies
 */
bool CustomRigidBody::findSeparationPlane(CustomRigidBody* rb_p)
{
  // compute a separation plane along each polygon of the first body
  for(int i = 0; i < this->structure.polygons.size(); ++i)
  {
    Plane sp = this->structure.polygons[i].getPlane();

    // check if all the vertices of the second body are outside of the separation plane
    for(int j = 0; j < rb_p->structure.vertices.size(); ++j)
    {
      double distance;
      Geometry::closestPointOfPlane(rb_p->structure.vertices[j].absPosition, sp, &distance);

      if(distance < 0)
        break;
      else if(j == rb_p->structure.vertices.size() - 1)
        return true;
    }
  }

  return false;
}

/**
 * Determine the exact contact point by integrating backward in time
 */
std::vector<Contact> CustomRigidBody::resolveInterPenetration(CustomRigidBody* rb_p, double dt, double tolerance)
{
  // find the distance between the two bodies
  Vector3 distance = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p);
  bool interPenetration = !this->findSeparationPlane(rb_p) && !rb_p->findSeparationPlane(this);

  std::cout << "d = " << distance.length() << " ip = " << interPenetration << std::endl;
	std::cout << *this << std::endl;

  // if bodies are within the tolerance area, compute the real contact points
  if(distance.length() < tolerance)
  {
    std::vector<Contact> vfContacts = Geometry::vertexFaceContacts(this, rb_p, tolerance);
    std::vector<Contact> eeContacts = Geometry::edgeEdgeContacts(this, rb_p, tolerance);
    std::cout << vfContacts.size() << " v/f and " << eeContacts.size() << " e/e" << std::endl;

    for(int i = 0; i < eeContacts.size(); ++i)
			vfContacts.push_back(eeContacts[i]);

    return vfContacts;
  }
  // if the bodies are too far apart, integrate forward in time
  else if(!interPenetration)
  {
	  std::cout << "going forward " << dt/100 << "ms" << std::endl;

		double sdt = dt / 100;

	  engine_pg->applyEnvironmentalForces(this, sdt);
	  engine_pg->applyEnvironmentalForces(rb_p, sdt);

    this->integrate(sdt);
    rb_p->integrate(sdt);

    return this->resolveInterPenetration(rb_p, dt, tolerance);
  }
  // else if the bodies are inter-penetrating, integrate backward in time
  else
  {
	  std::cout << "going backward " << dt/100 << "ms" << std::endl;

		double sdt = dt / 100;
	  engine_pg->applyEnvironmentalForces(this, sdt);
	  engine_pg->applyEnvironmentalForces(rb_p, sdt);

		this->reverseTime();
		rb_p->reverseTime();

    this->integrate(-sdt);
    rb_p->integrate(-sdt);

		this->reverseTime();
		rb_p->reverseTime();

//  exit(0);
    return this->resolveInterPenetration(rb_p, dt, tolerance);
  }
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

