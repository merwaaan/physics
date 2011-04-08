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
  else if(k == 97)
  {
	  Cube* c = new Cube(2);

	  c->setPosition((float)rand() / RAND_MAX * 8, 5, (float)rand() / RAND_MAX * 8);
	  c->position += Vector3(-4, 0, -4);
	  
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

void demoBalls()
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

void demoBoxes()
{
	Cube* c2 = new Cube(4);
  c2->setFixed(true);
  e->addRigidBody_p(c2);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addEnvironmentalForce_p(g);

  e->run();
}

int main(int argc, char** argv)
{
	srand(1);

  e = new Engine(&argc, argv, 0.0166);
  
  glutKeyboardFunc(&input);
  
  //testGeometry();
  //testGJK();
  //testTimeReversing();

  //demoBalls();
  demoBoxes();
  
  delete e;

  return 0;
}

