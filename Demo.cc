#include <iostream>
#include <GL/glut.h>

#include "Engine.h"
#include "Box.h"
#include "Sphere.h"

Engine* e;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    for(int i = 0; i < e->getBodyCount(); ++i)
	    e->reverseTime();
}

void compareRK4ToEuler()
{
  double dt = 0.0166;

  Sphere* s = new Sphere(1);
  s->prepare();
  Sphere* s2 = new Sphere(1);
  s2->prepare();

  s2->applyCenterForce(Vector3(10, 0, 0), dt);
  s2->integrate(0.1);

  for(int i = 1; i < 10; ++i)
  {
    s->applyCenterForce(Vector3(10, 0, 0), dt);
    s2->applyCenterForce(Vector3(10, 0, 0), dt);
    
    s->integrate(0.1);
    s2->integrate2(0.1);
    
    std::cout << 0.5 * 10  * i * i / 100<< " " << s->position.length() << " " << s2->position.length() << std::endl;
  }
 
}

void testTimeReversing()
{
  double dt = 1;

  Sphere* s = new Sphere(1);
  s->prepare();
  s->applyCenterForce(Vector3(0, -9.81, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), dt); s->integrate(dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), dt); s->integrate(dt); std::cout << *s << std::endl;

  std::cout << "reversing time..." << std::endl << std::endl;
  s->linearMomentum = -1 * s->linearMomentum;

  std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), -dt); s->integrate(-dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), -dt); s->integrate(-dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), -dt); s->integrate(-dt); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0), -dt); s->integrate(-dt); std::cout << *s << std::endl;
}

void demoSimpleBox()
{
	Cube* c = new Cube(2);
	//c->applyOffCenterForce(Vector3(0, 10, 5), 1, Vector3(1, 1, 1));
  c->angularMomentum = Vector3(0, 100, 100);
  e->addRigidBody_p(c);

  e->run();
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
  e->addForce_p(g);

  e->run();
}

int main(int argc, char** argv)
{
  e = new Engine(&argc, argv, 0.0166);
  
  glutKeyboardFunc(&input);

  //demoSimpleBox();
  //compareRK4ToEuler();
  //testTimeReversing();
  demoMultiBall();

  return 0;
}

