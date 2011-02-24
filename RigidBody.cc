#include "RigidBody.h"

RigidBody::RigidBody()
{
  // initial orientation (aligned with the axis)
  this->rotation(0, 0, 1);
  this->rotation(1, 1, 1);
  this->rotation(2, 2, 1);
  
  //Vector3 f(0, 300, 0);
  //applyCenterForce(f);

  Vector3 f(0, 10, 0);
  Vector3 poa(2, 2, 0);
  this->applyOffCenterForce(f, poa);
}

RigidBody::~RigidBody()
{
  for(int i = 0; i < this->structure.polygons.size(); ++i)
    delete[] this->structure.polygons[i].vertices_p;
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb)
{
  os << "position :" << rb.position << std::endl;
  os << "linear momentum :" << rb.linearMomentum << std::endl;
  os << "rotation :" << std::endl << rb.rotation;
  os << "angular momentum :" << rb.angularMomentum << std::endl;

  return os;
}

void RigidBody::clearAccumulators()
{
  this->accumulatedForces.reset();
  this->accumulatedTorques.reset();
}

void RigidBody::applyCenterForce(Vector3& force)
{
  this->accumulatedForces += force;
}

void RigidBody::applyOffCenterForce(Vector3& force, Vector3& poa)
{
  this->accumulatedForces += force;
  this->accumulatedTorques += poa ^ force;
}

void RigidBody::integrate(double t)
{
  // linear movement
  this->linearMomentum += this->accumulatedForces;
  Vector3 velocity = this->linearMomentum * this->structure.inverseMass;
  this->position += velocity * t;

  // angular movement
  std::cout << "________________________" << std::endl;
  std::cout << *this << std::endl;;
  this->angularMomentum += this->accumulatedTorques;
  Matrix3 inertia = (this->rotation * this->structure.inertiaTensor) * this->rotation.transpose();
  Vector3 angularVelocity = inertia * this->angularMomentum;
  std::cout << *this << std::endl;;
  this->rotation += (angularVelocity ^ this->rotation) * t;
  std::cout << *this << std::endl;;

  // normalize rotation matrix to avoid numerical drift
  //this->rotation = this->rotation.normalize();

  this->computeVerticesAbsolutePositions();

  this->clearAccumulators();
}

void RigidBody::computeVerticesAbsolutePositions()
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
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
  for(int i = 0; i < count; ++i)
    p.vertices_p[i] = getVertexById_p(ids[i]);

  this->structure.polygons.push_back(p);
}

void RigidBody::computeCenterOfMass()
{
  double totalMass = 0.0;
  
  for(int i = 0; i < this->structure.vertices.size(); ++i)
  {
    Vertex* v_p = &this->structure.vertices[i];

    this->position += v_p->localPosition * v_p->mass;  
    totalMass += v_p->mass;
  }

  this->structure.inverseMass = 1 / totalMass;

  this->position = this->position * this->structure.inverseMass;
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

