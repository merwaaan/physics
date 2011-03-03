#include <iostream>
#include <GL/glut.h>

#include "Engine.h"
#include "Box.h"
#include "Sphere.h"

Cube *c;

void input(unsigned char k, int x, int y)
{
  if(k == 32)
    c->applyOffCenterForce(Vector3(100, 0, 0), Vector3(0, 2, 3));
}

int main(int argc, char** argv)
{
  Engine e(&argc, argv, 0.001);

  /*c = new Cube(3);
  c->setPosition(Vector3(-3, 0, 0));
  e.addRigidBody_p(c);*/
  
  for(int i = -5; i < 5; i += 4)
    for(int j = -5; j < 5; j += 4)
    {
      Sphere* s2 = new Sphere(1);
      s2->setPosition(Vector3(i + 0.1 * j + 0.3 * i, i + 5, j + 1.5));
      e.addRigidBody_p(s2);
      std::cout << *s2 << std::endl;
    } 
  
  for(int i = -5; i < 5; i += 3)
    for(int j = -5; j < 5; j += 3)
    {
      Sphere* s2 = new Sphere(1);
      s2->setPosition(Vector3(i, i, j));
      s2->setFixed(true);
      e.addRigidBody_p(s2);
      std::cout << *s2 << std::endl;
    }
  
  /*Box* plane = new Box(10, 1, 10);
  plane->setPosition(Vector3(0, -6, 0));
  plane->setFixed(true);
  e.addRigidBody_p(plane);*/

  Force* g = new CenterForce(Vector3(0, -9.81, 0));
  e.addForce_p(g);

  glutKeyboardFunc(&input);

  e.run();

  return 0;
}

