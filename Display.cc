#include <GL/glut.h>

#include <sys/time.h>

#include "Display.h"
#include "Engine.h"

Display* display_pg = NULL;

Display::Display(int* argc, char** argv, int w, int h, Engine* engine_p) :
  drawBoundingBoxes(true),
  engine_p(engine_p)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(w, h);
  glutCreateWindow("Demo");

  glEnable(GL_DEPTH_TEST);
  
  // lighting settings
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glShadeModel(GL_SMOOTH);
  
  GLfloat lightModel[] = {0.3, 0.3, 0.3, 1.0};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightModel);

  GLfloat lightSpecular[] = {0.5, 0.5, 0.5, 1.0};
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
  
  GLfloat lightAmbient[] = {170.0/255, 162.0/255, 113.0/255, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
 
  GLfloat lightPosition[] = {5, 10, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  
  glutDisplayFunc(&update);
  glutPassiveMotionFunc(&mouse);

  this->camera.radius = 20;
  this->camera.angle = 45;
  this->camera.lastX = -1;

  display_pg = this;
}

Display::~Display()
{
}

void Display::run()
{
  this->startingTime = this->getAbsoluteTime();
  this->lastUpdateTime = 0;

  glutMainLoop();
}

double Display::getAbsoluteTime()
{
  struct timeval t;
  gettimeofday(&t, NULL);

  return t.tv_sec + (double)t.tv_usec / 1000000;
}

double Display::getLocalTime()
{
  return this->getAbsoluteTime() - this->startingTime;
}

void Display::setBoundingBoxesDrawn(bool draw)
{
  this->drawBoundingBoxes = draw;
}

bool Display::areBoundingBoxesDrawn()
{
  return this->drawBoundingBoxes;
}

void update()
{
  Engine* engine_p = display_pg->engine_p;
  if(display_pg->getLocalTime() > display_pg->lastUpdateTime + 0.01666) // ~60 FPS
  {
    // update the simulation
	  engine_p->update();

    // clear all
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClearDepth(1);
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
      6,
      cam_p->radius * sin(cam_p->angle),
      0, 0, 0,
      0, 1, 0);
    
    // draw each rigid body
    for(int i = 0; i < engine_p->getBodyCount(); ++i)
	    engine_p->getBody_p(i)->draw();
    
    glutSwapBuffers();

    display_pg->lastUpdateTime = display_pg->getLocalTime();
  }

  glutPostRedisplay();
}

void mouse(int x, int y)
{
  Camera* cam_p = &display_pg->camera;
  
  if(cam_p->lastX < 0)
    cam_p->lastX = x;

  cam_p->angle += 0.01 * (x - cam_p->lastX);

  cam_p->lastX = x;
}

