#include <iostream>

#include "Engine.h"
#include "Cube.h"

Cube *c;
Cube *p;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    c->applyOffCenterForce(Vector3(20, 0, 0), Vector3(0, 2, 3));
}

int main(int argc, char** argv)
{
  Engine e(&argc, argv, 0.3);

  c = new Cube(3);
  e.addRigidBody_p(c);
  
  p = new Cube(10);
  p->setPosition(Vector3(0, -10, 0));
  p->setFixed(true);
  e.addRigidBody_p(p);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e.addForce_p(g);

  e.setKeyboardCallback_p(&input);

  e.run();

  return 0;
}

