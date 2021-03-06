#include <GL/glut.h>

#include <sys/time.h>

#include "Display.h"
#include "Engine.h"

extern Engine* E;

Display::Display(int* argc, char** argv, int w, int h) :
  drawBoundingBoxes(false)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
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
  
  GLfloat lightAmbient[] = {100.0/255, 100.0/255, 100.0/255, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
 
  GLfloat lightPosition[] = {5, 10, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  
	glEnable(GL_COLOR_MATERIAL);
  
	glutDisplayFunc(&update);
  glutPassiveMotionFunc(&mouseMoved);

  this->camera.radius = 30;
  this->camera.angle = 90;
	this->camera.height = 10;
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

void Display::setCamera(double radius, double angle, double height)
{
	camera.radius = radius;
	camera.angle = angle;
	camera.height = height;
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
	// Update the simulation.
	if(E->needUpdate())
		E->update();

	// Display the simulation.
	double t = E->getLocalTime();
	if(t > E->getDisplay_p()->getLastDisplayTime() + 0.001)
  {
    // Clear all.
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
    // Set an orthogonal perspective.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50, 1, 1, 2000);

    // Set the camera.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Camera* cam_p = E->getDisplay_p()->getCamera_p();
		double angleRad = cam_p->angle * M_PI/180;
    gluLookAt(
      cam_p->radius * sin(angleRad),
      cam_p->height,
      cam_p->radius * cos(angleRad),
      0, 0, 0,
      0, 1, 0);
		
    // Draw each rigid body.
    for(int i = 0; i < E->getBodyCount(); ++i)
	    E->getBody_p(i)->draw();

		// Draw each constraint.
		for(int i = 0; i < E->getConstraintCount(); ++i)
			E->getConstraint_p(i)->draw();

    glutSwapBuffers();
    
    E->getDisplay_p()->setLastDisplayTime(t);
  }

  glutPostRedisplay();
}

void mouseMoved(int x, int y)
{
	Camera* cam_p = E->getDisplay_p()->getCamera_p();
  
  if(cam_p->lastX < 0)
    cam_p->lastX = x;

  cam_p->angle += 0.4 * (x - cam_p->lastX);
  cam_p->lastX = x;
}
