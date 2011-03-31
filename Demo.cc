#include <iostream>
#include <GL/glut.h>

#include "Box.h"
#include "Engine.h"
#include "Geometry.h"
#include "Sphere.h"

Engine* e;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
	  e->reverseTime();
}

void testGeometry()
{
  std::cout << "CLOSEST OF EDGE" << std::endl;
  Edge e = {Vector3(-1,0,0), Vector3(2,0,0)};
  std::cout << Geometry::closestPointOfEdge(Vector3(0,0,0), e) << std::endl;
  std::cout << Geometry::closestPointOfEdge(Vector3(1,4,0), e) << std::endl;
  std::cout << Geometry::closestPointOfEdge(Vector3(-4,2,3), e) << std::endl;

  std::cout << "CLOSEST OF PLANE" << std::endl;
  Plane p = {Vector3(0,0,0), Vector3(0,1,0)};
  std::cout << Geometry::closestPointOfPlane(Vector3(10,0,0), p) << std::endl;
  std::cout << Geometry::closestPointOfPlane(Vector3(0,10,0), p) << std::endl;
  std::cout << Geometry::closestPointOfPlane(Vector3(3,3,3), p) << std::endl;

  std::cout << "CLOSEST OF TRIANGLE" << std::endl;
  Triangle t = {Vector3(-1,0,0), Vector3(1,0,0), Vector3(0,1,0)};
  std::cout << Geometry::closestPointOfTriangle(Vector3(0,3,0), t) << std::endl;
  std::cout << Geometry::closestPointOfTriangle(Vector3(-3,0,0), t) << std::endl;
  std::cout << Geometry::closestPointOfTriangle(Vector3(-1,0.5,0), t) << std::endl;

  std::cout << "CLOSEST OF SPHERE" << std::endl;
  Sphere s(3);
  std::cout << Geometry::closestPointOfSphere(Vector3(10,0,0), s) << std::endl;
  std::cout << Geometry::closestPointOfSphere(Vector3(-10,0,0), s) << std::endl;
  std::cout << Geometry::closestPointOfSphere(Vector3(3,3,0), s) << std::endl;

  std::cout << "CLOSEST OF EDGE/EDGE" << std::endl;
  Vector3 c1, c2;
  Edge e1 = {Vector3(0,0,0), Vector3(10,0,0)};
  Edge e2 = {Vector3(5,5,0), Vector3(5,-5,0)};
  std::cout << c1 << c2 << Geometry::edgeEdgeDistance(e1, e2, &c1, &c2) << std::endl;
}

void testGJK()
{
  Cube* c1 = new Cube(2);
  c1->prepare();

  Cube* c2 = new Cube(2);
  c2->setPosition(5, 0, 0);
  c2->prepare();
  
  std::vector<Vector3> m = Geometry::minkowskiDifference(c1, c2);
  std::cout << "CONVEX HULL" << std::endl;
  for(int i = 0; i < m.size(); ++i)
    std::cout << m[i] << std::endl;

  std::cout << "DISTANCE" << Geometry::gjkDistanceBetweenPolyhedra(c1, c2) << std::endl;
}

void testTimeReversing()
{
  double dt = 1;
	double g = -1;

  Sphere* s = new Sphere(1);
  s->prepare();
	s->angularMomentum = Vector3(1, 0, 0);

	std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;

  std::cout << "reversing time..." << std::endl << std::endl;
  s->linearMomentum *= -1;
	s->angularMomentum *= -1;
	dt *= -1;

  std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
}

void demoMultiBall()
{
  int q = 6;

  for(int i = -q; i < q; i += 3)
    for(int j = -q; j < q; j += 3)
    {
      Sphere* s = new Sphere(1);
      s->setPosition(i + 0.3 * i, 5, j * 0.1 + j);
      e->addRigidBody_p(s);
    }

  for(int i = -q; i < q; i += 3)
    for(int j = -q; j < q; j += 3)
    {
      Sphere* s = new Sphere(1);
      s->setPosition(i, 0, j);
      s->setFixed(true);
      e->addRigidBody_p(s);
    }

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void demoSimpleBox()
{
	Cube* c = new Cube(2);
	c->applyOffCenterForce(Vector3(0, 10, 5), 1, Vector3(1, 1, 1));
  e->addRigidBody_p(c);

  e->run();
}

void demoMultiBox()
{
	Cube* c = new Cube(2);
	c->angularMomentum = Vector3(3, 3, 3);
	c->setPosition(2.5, 5, 0);
  e->addRigidBody_p(c);

	Cube* c2 = new Cube(4);
  c2->setFixed(true);
  e->addRigidBody_p(c2);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

int main(int argc, char** argv)
{
  e = new Engine(&argc, argv, 0.0166);
  
  glutKeyboardFunc(&input);
  
  //testGeometry();
  //testGJK();
  //testTimeReversing();

  //demoMultiBall();
  demoSimpleBox();
	//demoMultiBox();
  
  delete e;

  return 0;
}

