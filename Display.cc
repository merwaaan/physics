#include <GL/glut.h>

#include "Display.h"

#include "Engine.h"

Display* g_display_p = NULL;

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

  g_display_p = this;

  glutDisplayFunc(&update);
}

Display::~Display()
{
}

void Display::run()
{
  glutMainLoop();
}

Engine* Display::getEngine_p()
{
  return this->engine_p;
}

void update()
{
  Engine* engine_p = g_display_p->getEngine_p();

  if(engine_p->needUpdate())
  {
    // update the simulation
    engine_p->update();

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0, glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT), 1.0, 30.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);

    glColor3f(0.0, 0.0, 0.0);

    int i, j, k;
    // for each rigid body
    for(i = 0; i < engine_p->getBodyCount(); ++i)
    {
      RigidBody* rb_p = engine_p->getBody_p(i);

      // for each polygon
      for(j = 0; j < rb_p->getPolyCount(); ++j)
      {
        Polygon* poly_p = &(rb_p->structure.polygons[j]);

        Vector3 normal = poly_p->getNormal();

        glBegin(GL_POLYGON);

        // for each vertex
        for(k = 0; k < poly_p->size; ++k)
        {
          glNormal3f(normal.X(), normal.Y(), normal.Z());
          glVertex3d(poly_p->vertices_p[k]->absPosition.X(), poly_p->vertices_p[k]->absPosition.Y(), poly_p->vertices_p[k]->absPosition.Z());
        }

        glEnd();
      }  
    }

    glutSwapBuffers();
  }

  glutPostRedisplay();
}

