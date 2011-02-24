#include <iostream>

#include "Engine.h"
#include "Cube.h"

int main(int argc, char** argv)
{
  Engine e(&argc, argv, 0.3);

  Cube* c = new Cube(3);
  e.addRigidBody_p(c);
  
  /*Cube* p = new Cube(10);
  p->setPosition(Vector3(0, -100, 0));
  e.addRigidBody_p(p);*/

  //Force* g = new CenterForce(Vector3(0, -9.81, 0));
  //e.addForce_p(g);

  e.run();

  return 0;
}

