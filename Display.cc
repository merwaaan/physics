#include <GL/glut.h>

#include <sys/time.h>

#include "Display.h"
#include "Engine.h"

extern Engine* E;

Display::Display(int* argc, char** argv, int w, int h) :
  drawBoundingBoxes(false)
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

  GLfloat lightSpecular[] = {0.1, 0.1, 0.1, 1.0};
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
  
  GLfloat lightAmbient[] = {200.0/255, 200.0/255, 200.0/255, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
 
  GLfloat lightPosition[] = {5, 10, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  
  glutDisplayFunc(&update);
  glutPassiveMotionFunc(&mouse);

  this->camera.radius = 50;
  this->camera.angle = 45;
  this->camera.lastX = -1;
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
  if(E->getDisplay_p()->getLocalTime() > E->getDisplay_p()->lastUpdateTime + 0.01666) // ~60 FPS
  {
    // update the simulation
	  E->update();

    // clear all
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
    // set an orthogonal perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
		int h = 10;
    glOrtho(-h, h, -h, h, 1, 200);

    // place the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Camera* cam_p = &E->getDisplay_p()->camera;
    gluLookAt(
      cam_p->radius * cos(cam_p->angle),
      6,
      cam_p->radius * sin(cam_p->angle),
      0, 0, 0,
      0, 1, 0);
		
    // draw each rigid body
    for(int i = 0; i < E->getBodyCount(); ++i)
	    E->getBody_p(i)->draw();

		// draw each constraint
		for(int i = 0; i < E->getConstraintCount(); ++i)
			E->getConstraint_p(i)->draw();

    glutSwapBuffers();
    
    E->getDisplay_p()->lastUpdateTime = E->getDisplay_p()->getLocalTime();
  }

  glutPostRedisplay();
}

void mouse(int x, int y)
{
  Camera* cam_p = &E->getDisplay_p()->camera;
  
  if(cam_p->lastX < 0)
    cam_p->lastX = x;

  cam_p->angle += 0.01 * (x - cam_p->lastX);

  cam_p->lastX = x;
}

