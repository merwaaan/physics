#include <iostream>
#include <GL/glut.h>

#include "Box.h"
#include "Engine.h"
#include "Geometry.h"
#include "Sphere.h"

Engine* e;

void input(unsigned char k, int x, int y)
{
  if(k == 97)
  {
	  Cube* c = new Cube(2);
	  c->setPosition((float)rand() / RAND_MAX * 8 - 4, 5, (float)rand() / RAND_MAX * 8 - 4);
	  c->prepare();
	  e->addRigidBody_p(c);
  }
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
  Cube* c1 = new Cube(2);
  c1->setPosition(0, 0, 0);
  c1->prepare();

  Cube* c2 = new Cube(2);
  c2->setPosition(5, 0, 0);
  c2->prepare();
  
  std::cout << "DISTANCE" << Geometry::gjkDistanceBetweenPolyhedra(c1, c2) << std::endl;
}

void testTimeReversing()
{
  double dt = 1;
	double g = -1;

  Sphere* s = new Sphere(1);
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

void demoRope()
{
	Sphere* s;
	Constraint* c;
	Sphere* prev;

	s = new Sphere(1);
	s->setFixed(true);
	e->addRigidBody_p(s);	
	prev = s;

	for(int i = 0; i < 15; ++i)
	{
		s = new Sphere(1);
		s->setPosition(i*1.3, 3 + i * 3, 0);
		e->addRigidBody_p(s);

		c = new DistanceConstraint(s, prev, 3);
		e->addConstraint_p(c);

		prev = s;
	}

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

	e->run();
}

void demoPendulum()
{
	Constraint* c;

	Sphere* sf = new Sphere(0.1);
	sf->setPosition(-2, 6, 0);
	sf->setFixed(true);
	e->addRigidBody_p(sf);

	Sphere* sp = new Sphere(1);
	sp->setPosition(-8, 6, 0);
	e->addRigidBody_p(sp);
	
	c = new DistanceConstraint(sf, sp, 6);
	e->addConstraint_p(c);

	for(int i = 0; i < 4; ++i)
	{
		sf = new Sphere(0.1);
		sf->setPosition(i*2, 6, 0);
		sf->setFixed(true);
		e->addRigidBody_p(sf);

		sp = new Sphere(1);
		sp->setPosition(i*2, 0, 0);
		e->addRigidBody_p(sp);	

		c = new DistanceConstraint(sf, sp, 6);
		e->addConstraint_p(c);
	}

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

	e->run();
}

void demoBalls()
{
  for(int i = -2; i < 2; ++i)
    for(int j = -2; j < 2; ++j)
    {
      Sphere* s = new Sphere(1);
      s->setPosition(i * 3 + 0.1 * i, 5, j * 3 + 0.2 * j);
      e->addRigidBody_p(s);
    }

	for(int i = -4; i < 4; ++i)
		for(int j = -4; j < 4; ++j)
    {
      Sphere* s = new Sphere(1);
      s->setPosition(i * 3, 0, j * 3);
      s->setFixed(true);
      e->addRigidBody_p(s);
    }

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void demoMixed()
{
	Cube* c = new Cube(2);
	c->setPosition(0, 5, 0);
  e->addRigidBody_p(c);

	Sphere* s = new Sphere(1);
	s->setPosition(0, 4, 4);
  e->addRigidBody_p(s);

	Box* sol = new Box(30, 0.5, 30);
	sol->setPosition(0, -2.5, 0);
  sol->setFixed(true);
  e->addRigidBody_p(sol);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

void demoBoxes()
{
	Cube* c = new Cube(4);
  c->setFixed(true);
  e->addRigidBody_p(c);

	Box* sol = new Box(30, 0.5, 30);
	sol->setPosition(0, -2.5, 0);
  sol->setFixed(true);
  e->addRigidBody_p(sol);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

int main(int argc, char** argv)
{
	srand(1);//123);

  e = new Engine(&argc, argv, 0.01);
  
  glutKeyboardFunc(&input);
  
  //testGeometry();
  //testGJK();
  //testTimeReversing();
	
  //demoRope();
	//demoBalls();
  //demoPendulum();
	demoMixed();
  demoBoxes();
  
  delete e;

  return 0;
}
