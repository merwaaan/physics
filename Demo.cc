#include <iostream>
#include <GL/glut.h>

#include "Box.h"
#include "Cylinder.h"
#include "Engine.h"
#include "Geometry.h"
#include "Sphere.h"

Engine* e;

float randomf()
{
	return (float)rand() / RAND_MAX;
}

/**
 * TESTS
 */

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
  Sphere s(3, 1);
  std::cout << Geometry::closestPointOfSphere(Vector3(10,0,0), s) << std::endl;
  std::cout << Geometry::closestPointOfSphere(Vector3(-10,0,0), s) << std::endl;
  std::cout << Geometry::closestPointOfSphere(Vector3(3,3,0), s) << std::endl;

  std::cout << "CLOSEST OF EDGE/EDGE" << std::endl;
  Vector3 c1, c2;
  Edge e1 = {Vector3(0,0,0), Vector3(10,0,0)};
  Edge e2 = {Vector3(5,5,0), Vector3(5,-5,0)};
  std::cout << c1 << c2 << Geometry::edgeEdgeDistance(e1, e2, &c1, &c2) << std::endl;

	std::cout << "TETRAHEDRON RESTRUCTURATION" << std::endl;
	Vector3 a(-1, 0, 0), b(1, 0, 0), c(0, 0, -1), d(0, 1, 0);
	Triangle t1 = {a, b, c}, t2 = {a, b, d}, t3 = {d, c, b}, t4 = {d, c, a};
	Tetrahedron te = {t1, t2, t3, t4};
	std::vector<Triangle> triangles = te.getTriangles();
	for(int i = 0; i < 4; ++i)
		std::cout << triangles[i].getPlane().normal << std::endl;

	std::cout << "INSIDE TETRAHEDRON?" << std::endl;
	std::cout << Geometry::isInsideTetrahedron(Vector3(-1, 0, 0), te) << std::endl;
	std::cout << Geometry::isInsideTetrahedron(Vector3(0.5, 0.1, 0), te) << std::endl;
	std::cout << Geometry::isInsideTetrahedron(Vector3(10, 0, 0), te) << std::endl;

	std::cout << "CLOSEST POINT OF TETRAHEDRON" << std::endl;
	std::cout << Geometry::closestPointOfTetrahedron(Vector3(-10, 0, 0), te) << std::endl;
	std::cout << Geometry::closestPointOfTetrahedron(Vector3(10, 0, 0), te) << std::endl;
	std::cout << Geometry::closestPointOfTetrahedron(Vector3(0, 10, 0), te) << std::endl;
	std::cout << Geometry::closestPointOfTetrahedron(Vector3(0.25, 0.25, 1), te) << std::endl;
	std::cout << Geometry::closestPointOfTetrahedron(Vector3(0, 0.25, -0.25), te) << std::endl;
}

void testGJK()
{
  Cube* c1 = new Cube(2,1);
  c1->setPosition(0, 0, 0);
  c1->prepare();

  Cube* c2 = new Cube(2,1);
  c2->setPosition(5, 0, 0);
  c2->prepare();
  
  std::cout << "DISTANCE" << Geometry::gjkDistance(c1, c2) << std::endl;
}

void testTimeReversing()
{
  double dt = 1;
	double g = -1;

  Sphere* s = new Sphere(1, 1);
  s->prepare();
  s->setAngularMomentum(1, 0, 0);

	std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;

  std::cout << "reversing time..." << std::endl << std::endl;
  s->setLinearMomentum(s->getLinearMomentum() * -1);
  s->setAngularMomentum(s->getAngularMomentum() * -1);
	dt *= -1;

  std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, g, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
}

/**
 * DEMOS
 */

void inputBasic(unsigned char k, int x, int y)
{
	if(k == 115) // S
		e->getDisplay_p()->getCamera_p()->radius += 1;
	else if(k == 122) // Z
		e->getDisplay_p()->getCamera_p()->radius -= 1;
}

void inputDemoBoxes(unsigned char k, int x, int y)
{
	inputBasic(k, x, y);

  if(k == 97) // A
  {
	  //Box* c = new Box(randomf()*10, randomf()*10, randomf()*10, 10);
		Cylinder* c = new Cylinder(1, 6, 5, 1);
		c->setPosition(randomf()*50-25, 10, randomf()*50-25);
		c->prepare();
	  e->addRigidBody_p(c);
  }
}

void demoBoxes()
{
	srand(12);
  glutKeyboardFunc(&inputDemoBoxes);

	Cube* c = new Cube(4,1);
  c->setFixed(true);
  e->addRigidBody_p(c);

	Box* floor = new Box(50, 3, 50, 1);
	floor->setPosition(0, -3.5, 0);
  floor->setFixed(true);
  e->addRigidBody_p(floor);

  Force* g = new Gravity(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void inputDemoZeroG(unsigned char k, int x, int y)
{
	inputBasic(k, x, y);

  if(k == 97)
  {
	  Box* c = new Box(randomf()*10, randomf()*10, randomf()*10, 10);
	  c->setPosition(randomf()*50-25, 0, randomf()*50-25);
		c->prepare();
		c->applyOffCenterForce(
			Vector3(randomf()*10-5, randomf()*10-5, randomf()*10-5),
			Vector3(randomf()*10-5, randomf()*10-5, randomf()*10-5),
			1);
	  e->addRigidBody_p(c);
  }
}

void demoZeroG()
{
	srand(12);  
  glutKeyboardFunc(&inputDemoZeroG);

  e->run();
}

void inputDemoPachinko(unsigned char k, int x, int y)
{
	inputBasic(k, x, y);

  if(k == 97) // A
  {
	  Cube* c = new Cube(3, 3);
	  c->setPosition(randomf()*40-20, 30, 10);
		c->prepare();
	  e->addRigidBody_p(c);
  }
}

void demoPachinko()
{
	srand(1);
  glutKeyboardFunc(&inputDemoPachinko);

	e->getDisplay_p()->setCamera(80, -30);

	// Wall.
	Box* wall = new Box(50, 50, 5, 1);
	wall->setFixed(true);
	e->addRigidBody_p(wall);

	// Pins (odd)
	for(int i = 0; i < 5; ++i)
	{
		if(i % 2) continue;

		for(int j = 0; j < 5; ++j)
		{
			Box* pin = new Box(1, 1, 20, 1);
			pin->setPosition(j * 10 - 25 + 5, i * 10 - 25 + 5, 10);
			pin->setFixed(true);
			e->addRigidBody_p(pin);
		}
	}

	// Pins (even)
	for(int i = 0; i < 5; ++i)
	{
		if(!(i % 2)) continue;

		for(int j = 0; j < 4; ++j)
		{
			Box* pin = new Box(1, 1, 20, 1);
			pin->setPosition(j * 10 - 25 + 10, i * 10 - 25 + 5, 10);
			pin->setFixed(true);
			e->addRigidBody_p(pin);
		}
	}

  Force* g = new Gravity(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void inputDemoStairs(unsigned char k, int x, int y)
{
	inputBasic(k, x, y);

  if(k == 97) // A
  {
	  Cube* c = new Cube(3, 3);
	  c->setPosition(randomf()*20-10, 10, 5 + randomf()*5);
		c->prepare();
	  e->addRigidBody_p(c);
  }
	if(k == 113) // Q
  {
	  Sphere* s = new Sphere(1, 10);
	  s->setPosition(randomf()*20-10, 10, 5 + randomf()*5);
		s->prepare();
	  e->addRigidBody_p(s);
  }
}

void demoStairs()
{
	srand(100);
  glutKeyboardFunc(&inputDemoStairs);

	Box* c = NULL;
	int height = 7;
	int steps = 5;
	int stepSize = 2;

	for(int i = 0; i < steps; ++i)
	{
		c = new Box(20, (steps-i) * (double)height/steps, stepSize, 1);
		c->setPosition(0, c->getHeight()/2, 10-i*stepSize);
		c->setFixed(true);
		e->addRigidBody_p(c);
	}
			
	Box* floor = new Box(20, 0.5, 50, 1);
	floor->setPosition(0, 0, 0);
  floor->setFixed(true);
  e->addRigidBody_p(floor);

  Force* g = new Gravity(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void inputDemoRope(unsigned char k, int x, int y)
{
	inputBasic(k, x, y);
}

void demoRope()
{
	glutKeyboardFunc(&inputDemoRope);

	Sphere* s;
	Constraint* c;
	Sphere* prev;

	s = new Sphere(1, 1);
	s->setFixed(true);
	e->addRigidBody_p(s);	
	prev = s;

	for(int i = 0; i < 15; ++i)
	{
		s = new Sphere(1, 1);
		s->setPosition(i*3, 3 + i * 3, 0);
		e->addRigidBody_p(s);

		c = new DistanceConstraint(s, prev, 3);
		e->addConstraint_p(c);

		prev = s;
	}

  Force* g = new Gravity(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

	e->run();
}

int main(int argc, char** argv)
{
  e = new Engine(&argc, argv, 0.01);
  
  //testGeometry();
  //testGJK();
  //testTimeReversing();

	//demoZeroG();
  demoBoxes();
	demoPachinko();
	//demoStairs();
  //demoRope();
  
  delete e;

  return 0;
}
