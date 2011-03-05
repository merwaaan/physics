#include <iostream>
#include <GL/glut.h>

#include "Engine.h"
#include "Box.h"
#include "Sphere.h"

Engine* e;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    e->reverseTime();
}

void compareRK4ToEuler()
{
  Sphere* s = new Sphere(1);
  s->prepare();
  Sphere* s2 = new Sphere(1);
  s2->prepare();

  s2->applyCenterForce(Vector3(10, 0, 0));
  s2->integrate(0.1);

  for(int i = 1; i < 10; ++i)
  {
    s->applyCenterForce(Vector3(10, 0, 0));
    s2->applyCenterForce(Vector3(10, 0, 0));
    
    s->integrate(0.1);
    s2->integrate2(0.1);

    s->clearAccumulators();
    s2->clearAccumulators();
    
    std::cout << 0.5 * 10  * i * i / 100<< " " << s->position.length() << " " << s2->position.length() << std::endl;
  }
 
}

void testTimeReversing()
{
  Sphere* s = new Sphere(1);
  s->prepare();
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(0.0166); s->clearAccumulators(); std::cout << *s << std::endl;

  std::cout << "reversing time..." << std::endl;
  s->linearMomentum = -1 * s->linearMomentum;

  std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(-0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(-0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(-0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
  s->applyCenterForce(Vector3(0, -9.81, 0)); s->integrate(-0.0166); s->clearAccumulators(); std::cout << *s << std::endl;
}

int main(int argc, char** argv)
{
  e = new Engine(&argc, argv, 0.0166);

  /*Cube* c = new Cube(3);
  c->setPosition(Vector3(-3, 0, 0));
  e->addRigidBody_p(c);*/
 
  //testTimeReversing();

  Sphere* s = new Sphere(1);
  s->setPosition(0, 5, 0);
  e->addRigidBody_p(s);

  Sphere* s2 = new Sphere(1);
  s2->setFixed(true);
  e->addRigidBody_p(s2);
  
  /*for(int i = 0; i < 10; i += 3)
  {
    Sphere* s2 = new Sphere(1);
    s2->setPosition(i, 0, 0);
    s2->setFixed(true);
    e->addRigidBody_p(s2);
  }*/

  /*Box* plane = new Box(10, 1, 10);
  plane->setPosition(Vector3(0, -6, 0));
  plane->setFixed(true);
  e->addRigidBody_p(plane);*/

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e->addForce_p(g);

  glutKeyboardFunc(&input);

  e->run();

  return 0;
}

