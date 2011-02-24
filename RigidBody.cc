#include "RigidBody.h"

RigidBody::RigidBody()
{
  // initial orientation (aligned with the axis)
  this->orientation(0, 0, 1);
  this->orientation(1, 1, 1);
  this->orientation(2, 2, 1);
  
  Vector3 f(30, 0, 0);
  Vector3 poa(0, 2, 1);
  this->applyOffCenterForce(f, poa);
}

RigidBody::~RigidBody()
{
  for(int i = 0; i < this->structure.polygons.size(); ++i)
    delete[] this->structure.polygons[i].vertices_p;
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb)
{
  os << "position " << rb.position << std::endl;
  os << "linear momentum " << rb.linearMomentum << std::endl;
  os << "orientation " << std::endl << rb.orientation;
  os << "angular momentum " << rb.angularMomentum << std::endl;

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
  this->angularMomentum += this->accumulatedTorques;
  Matrix3 inverseInertia = this->orientation * this->structure.inverseInertiaTensor * this->orientation.transpose();
  Vector3 angularVelocity = inverseInertia * this->angularMomentum;
  this->orientation += (angularVelocity.toStarMatrix() * this->orientation) * t;

  // normalize the orientation matrix to avoid numerical drift
  this->orientation = this->orientation.normalize();
  
  std::cout << *this << std::endl;
  //usleep(1000000);

  this->computeVerticesAbsolutePositions();

  this->clearAccumulators();
}

void RigidBody::computeVerticesAbsolutePositions()
{
  for(int i = 0; i < this->structure.vertices.size(); ++i)
    this->structure.vertices[i].absPosition = this->orientation * this->structure.vertices[i].localPosition + this->position;
}

void RigidBody::setPosition(Vector3 position)
{
  this->position = position;
}

void RigidBody::setOrientation(Matrix3 orientation)
{
  this->orientation = orientation;
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

void RigidBody::prepare()
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

