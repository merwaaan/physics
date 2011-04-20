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
  this->lastDisplayTime = 0;

  glutMainLoop();
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
	double t = E->getLocalTime();

	// Update the simulation.
	if(t > E->lastUpdateTime + E->getTimeStep())
		E->update();

	// Display the simulation.
  if(t > E->getDisplay_p()->lastDisplayTime + 0.001)
  {
    // Clear all.
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
    // Set an orthogonal perspective.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
		int h = 10;
    glOrtho(-h, h, -h, h, 1, 200);

    // Set the camera.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Camera* cam_p = &E->getDisplay_p()->camera;
    gluLookAt(
      cam_p->radius * cos(cam_p->angle),
      6,
      cam_p->radius * sin(cam_p->angle),
      0, 0, 0,
      0, 1, 0);
		
    // Draw each rigid body.
    for(int i = 0; i < E->getBodyCount(); ++i)
	    E->getBody_p(i)->draw();

		// Draw each constraint.
		for(int i = 0; i < E->getConstraintCount(); ++i)
			E->getConstraint_p(i)->draw();

    glutSwapBuffers();
    
    E->getDisplay_p()->lastDisplayTime = E->getLocalTime();
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

