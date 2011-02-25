#include <iostream>
#include <GL/glut.h>

#include "Engine.h"
#include "Box.h"
#include "Sphere.h"

Cube *c;
Sphere* s;
Box* plane;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    c->applyOffCenterForce(Vector3(100, 0, 0), Vector3(0, 2, 3));
}

int main(int argc, char** argv)
{
  Engine e(&argc, argv, 0.01);

  c = new Cube(3);
  c->setPosition(Vector3(-3, 0, 0));
  e.addRigidBody_p(c);
  
  s = new Sphere(1.5);
  s->setPosition(Vector3(3, 0, 0));
  e.addRigidBody_p(s);

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

