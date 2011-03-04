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

int main(int argc, char** argv)
{
  e = new Engine(&argc, argv, 0.01);

  /*Cube* c = new Cube(3);
  c->setPosition(Vector3(-3, 0, 0));
  e->addRigidBody_p(c);*/
 
  Sphere* s = new Sphere(1);
  s->setPosition(1.8, 5, 0);
  e->addRigidBody_p(s);
  
  Sphere* s2 = new Sphere(1);
  s2->setPosition(0, 0, 0);
  s2->setFixed(true);
  e->addRigidBody_p(s2);

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

