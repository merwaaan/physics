#include "RigidBody.h"

RigidBody::RigidBody()
{
  // initial orientation (aligned with the axis)
  this->rotation(0, 0, 1);
  this->rotation(1, 1, 1);
  this->rotation(2, 2, 1);

  this->velocity.X(0.1);
  this->velocity.Y(1);
}

RigidBody::~RigidBody()
{
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb)
{
  os << "position :" << rb.position << std::endl;
  os << "velocity :" << rb.velocity << std::endl;
  os << "rotation :" << std::endl << rb.rotation;
  os << "angular velocity :" << rb.angularVelocity << std::endl;

  return os;
}

void RigidBody::applyForce(Vector3& force)
{
  this->accumulatedForces += force;
}

void RigidBody::clearAccumulatedForces()
{
  this->accumulatedForces.X(0);
  this->accumulatedForces.Y(0);
  this->accumulatedForces.Z(0);
}

void RigidBody::integrate(double t)
{
  // linear movement
  Vector3 acceleration = this->accumulatedForces * this->inverseMass;
  this->velocity += acceleration * t;
  //this->position += this->velocity * t;

  // angular movement
  //Vector3 angularAcceleration;
  this->angularVelocity = Vector3(0.01, 0.01, 0);
  this->rotation = this->rotation + (this->angularVelocity ^ this->rotation);

  this->computeVerticesAbsolutePositions();
}

void RigidBody::computeVerticesAbsolutePositions()
{
  int i;
  for(i = 0; i < this->structure.vertices.size(); ++i)
    this->structure.vertices[i].absPosition = this->rotation * this->structure.vertices[i].localPosition + this->position;
}

void RigidBody::addVertex(int id, double x, double y, double z, double m)
{
  Vertex v;
  v.id = id;
  v.localPosition = Vector3(x, y, z);
  v.mass = m;

  this->structure.vertices.push_back(v);
}

void RigidBody::addPolygon(int count, int* ids)
{
  Polygon p;
  p.size = count;
  p.vertices_p = new Vertex*[count];

  // link all referenced vertices with a polygon
  int i;
  for(i = 0; i < count; ++i)
    p.vertices_p[i] = getVertexById_p(ids[i]);

  this->structure.polygons.push_back(p);
}

void RigidBody::computeCenterOfMass()
{
  double totalMass = 0.0;
  
  int i;
  for(i = 0; i < this->structure.vertices.size(); ++i)
  {
    Vertex* v_p = &this->structure.vertices[i];

    this->position += v_p->localPosition * v_p->mass;  
    totalMass += v_p->mass;
  }

  this->inverseMass = 1 / totalMass;

  this->position = this->position * this->inverseMass;
}

Vertex* RigidBody::getVertexById_p(int id)
{
  int i;
  for(i = 0; i < this->structure.vertices.size(); ++i)
    if(this->structure.vertices[i].id == id)
      return &(this->structure.vertices[i]);

  return NULL;
}

int RigidBody::getPolyCount()
{
  return this->structure.polygons.size();
}

