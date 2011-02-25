#include <iostream>
#include <GL/glut.h>

#include "Engine.h"
#include "Box.h"

Cube *c;
Box *plane;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    c->applyOffCenterForce(Vector3(100, 0, 0), Vector3(0, 2, 3));
}

int main(int argc, char** argv)
{
  Engine e(&argc, argv, 0.01);

  c = new Cube(3);
  e.addRigidBody_p(c);
  
  plane = new Box(10, 1, 10);
  plane->setPosition(Vector3(0, -6, 0));
  plane->setFixed(true);
  e.addRigidBody_p(plane);

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e.addForce_p(g);

  glutKeyboardFunc(&input);

  e.run();

  return 0;
}

