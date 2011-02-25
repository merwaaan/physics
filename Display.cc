#include <GL/glut.h>

#include "Display.h"

#include "Engine.h"

Display* display_pg = NULL;

Display::Display(int* argc, char** argv, int w, int h, Engine* engine_p) :
  engine_p(engine_p)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(w, h);
  glutCreateWindow("Demo");

  glEnable(GL_DEPTH_TEST);
  
  // lighting settings
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glShadeModel(GL_SMOOTH);
  
  GLfloat lightModel[] = {0.2, 0.2, 0.2, 1.0};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightModel);

  GLfloat lightSpecular[] = {0.1, 0.1, 0.1, 1.0};
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
 
  GLfloat lightAmbient[] = {170.0/255, 162.0/255, 113.0/255, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);

  GLfloat lightPosition[] = {0.0, -3.0, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  glutDisplayFunc(&update);
  glutPassiveMotionFunc(&mouse);

  this->camera.radius = 10;

  display_pg = this;
}

Display::~Display()
{
}

void Display::run()
{
  glutMainLoop();
}

void update()
{
  Engine* engine_p = display_pg->engine_p;

  if(engine_p->needUpdate())
  {
    // update the simulation
    engine_p->update();

    // clear all
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // set an orthogonal perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10, 10, -10, 10, 1, 100);

    // place the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Camera* cam_p = &display_pg->camera;
    gluLookAt(
      cam_p->radius * cos(cam_p->angle),
      4,
      cam_p->radius * sin(cam_p->angle),
      0, 0, 0,
      0, 1, 0);

    glColor3f(0.0, 0.0, 0.0);

    // draw each rigid body
    for(int i = 0; i < engine_p->getBodyCount(); ++i)
      engine_p->getBody_p(i)->draw();

    glutSwapBuffers();
  }

  glutPostRedisplay();
}

void mouse(int x, int y)
{
  Camera* cam_p = &display_pg->camera;

  cam_p->angle += 0.01 * (x - cam_p->lastX);

  cam_p->lastX = x;
}

