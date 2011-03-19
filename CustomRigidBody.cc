#include "CustomRigidBody.h"

#include <GL/glut.h>
#include <limits>

#include "Display.h"
#include "Engine.h"
#include "Sphere.h"

extern Display* display_pg;
extern Engine* engine_pg;

/**
 * Return the best support point along a direction
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

void CustomRigidBody::addVertex(int id, double x, double y, double z, double m)
{
  CustomVertex v;
  v.id = id;
  v.localPosition = Vector3(x, y, z);
  v.mass = m;

  this->structure.vertices.push_back(v);
}

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
}

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

void CustomRigidBody::integrate(double t)
{
  RigidBody::integrate(t);

  this->computeVerticesAbsolutePositions();
  this->computeBoundingBox();
}

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

void CustomRigidBody::computeVerticesAbsolutePositions()
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
    this->structure.vertices[i].absPosition = this->orientation * this->structure.vertices[i].localPosition + this->position;
}

Contact* CustomRigidBody::isCollidingWith(RigidBody* rb_p, double dt)
{
  return rb_p->isCollidingWith(this, dt);
}

Contact* CustomRigidBody::isCollidingWith(Sphere* s_p, double dt)
{
  return NULL;
}

Contact* CustomRigidBody::isCollidingWith(CustomRigidBody* rb_p, double dt)
{
  double tolerance = 0.01;

  // if at least one separation plane exists, there is no collision
  if(this->findSeparationPlane(rb_p) || rb_p->findSeparationPlane(this))
  {
    std::cout << "separation plane found" << std::endl;
    usleep(1000000);
    return NULL;
  }

  std::cout << "no separation plane" << std::endl;
    usleep(1000000);
  return this->resolveInterPenetration(rb_p, dt, tolerance);
}

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
      std::cout << i << " " << distance << std::endl;
      if(distance <= 0)
        break;
      else if(j == rb_p->structure.vertices.size() - 1)
      {
        std::cout << sp.point << sp.normal << std::endl;
        return true;
      }
    }
  }

  return false;
}

Contact* CustomRigidBody::resolveInterPenetration(CustomRigidBody* rb_p, double dt, double tolerance)
{
  // find the distance between the two bodies
  Vector3 distance = Geometry::gjkDistanceBetweenPolyhedra(this, rb_p);
  std::cout << distance.length() << tolerance << std::endl;
  usleep(1000000);
  // if the bodies are too far apart, integrate forward in time
  if(distance.length() > tolerance)
  {
    std::cout << "going forward" << std::endl;
    this->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    rb_p->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    this->integrate(dt / 2);
    rb_p->integrate(dt / 2);
    
    return this->resolveInterPenetration(rb_p, dt / 2, tolerance);
  }
  // else if the bodies are inter-penetrating, integrate backward in time
  else if(distance.length() < tolerance)
  {
    std::cout << "going backward" << std::endl;
    this->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    rb_p->applyCenterForce(Vector3(0, -9.81, 0), dt / 2);
    engine_pg->reverseTime();
    this->integrate(-dt / 2);
    rb_p->integrate(-dt / 2);
    engine_pg->reverseTime();

    return this->resolveInterPenetration(rb_p, dt / 2, tolerance);
  }
  // else if bodies are within the tolerance zone, compute the real contact points
  else
  {
    std::cout << "OK!" << std::endl;
    Contact* contact_p = new Contact;

    contact_p->a = this;
    contact_p->b = rb_p;
    contact_p->position = Vector3(0,2,0);//contactingVertices[0].absPosition;
    contact_p->normal = Vector3(0,1,0);//(rb_p->position - contactingVertices[0].absPosition).normalize();

    return contact_p;
  }
}

CustomVertex* CustomRigidBody::getVertexById_p(int id)
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
    if(this->structure.vertices[i].id == id)
      return &(this->structure.vertices[i]);

  return NULL;
}

