/* DstarDraw.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 * v1.0 modified by Stefan Haase (stefan.haase@dfki.de)
 */

#ifdef MACOS
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h> 
#else
#include <GL/glut.h>
#include <GL/gl.h> 
#include <GL/glu.h>
#endif


#include <stdio.h>
#include <unistd.h>
#include "DStarLite.h"

int hh, ww;

int window; 
dstar_lite::DStarLite *dstar;

int scale = 6;
int mbutton = 0;
int mstate = 0;

bool b_autoreplan = true;

bool replan = true;

void InitGL(int Width, int Height)
{
  hh = Height;
  ww = Width;

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);	

  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,Width,0,Height,-100,100);
  glMatrixMode(GL_MODELVIEW);

}

void ReSizeGLScene(int Width, int Height)
{
  hh = Height;
  ww = Width;

  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,Width,0,Height,-100,100);
  glMatrixMode(GL_MODELVIEW);

}

int counter_reset = 0;
int max_reset = 1;

void DrawGLScene()
{

  usleep(1000);

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  glLoadIdentity();
  glPushMatrix();

  glScaled(scale,scale,1);

  if (b_autoreplan && replan) {
    if(dstar->replan()) {
      replan = false;
    } else {
      fprintf(stderr, "Path could not be found, goal will be reset.\n");
      dstar->resetGoal(); 
      counter_reset++;
      if(counter_reset >= max_reset) {
        fprintf(stderr, "Goal has been reset %d times, no path to goal could be found\n", max_reset);
        replan = false;
      }
    }
  }

  dstar->draw();
  
  glPopMatrix();
  glutSwapBuffers();

}


void keyPressed(unsigned char key, int x, int y) 
{
  double cost = 0;

  usleep(100);

  y = hh -y+scale/2;
  x += scale/2;

  switch(key) {
  case 'q':
  case 'Q':
    glutDestroyWindow(window); 
    exit(0);
    break;
  case 'r':
  case 'R':
    dstar->replan();
    break;
  case 'a':
  case 'A':
    b_autoreplan = !b_autoreplan;
    break;
  case 'c':
  case 'C':
    dstar->init(40,50,140, 90);
    dstar->replan();
    break;
  case '+':
    // Increments the cost of the mouse-over-cell.
    dstar->getCost(x/scale, y/scale, cost);
    dstar->updateCell(x/scale, y/scale, cost+1);
    replan = true;
    break;
  case '-':
    // Decrements the cost of the mouse-over-cell.
    // < 0 is an obstacle.
    dstar->getCost(x/scale, y/scale, cost);
    dstar->updateCell(x/scale, y/scale, cost-1);
    replan = true;
    break;
  case 'm':
  case 'M':
    dstar->createMap(100,100,1,10, dstar_lite::DStarLite::MAP_GRADIENT); // Hills.
    //dstar->createMap(150,150,1,2); // Realistic random field.
    dstar->replan();
    break;
  }
}

void mouseFunc(int button, int state, int x, int y) {
  
  y = hh -y+scale/2;
  x += scale/2;

  mbutton = button;

  if ((mstate = state) == GLUT_DOWN) {
    if (button == GLUT_LEFT_BUTTON) {
      dstar->updateCell(x/scale, y/scale, -1);
    } else if (button == GLUT_RIGHT_BUTTON) {
      dstar->updateStart(x/scale, y/scale);
    } else if (button == GLUT_MIDDLE_BUTTON) {
      dstar->updateGoal(x/scale, y/scale);
    }
    replan = true;
  }
}

void mouseMotionFunc(int x, int y)  {

  y = hh -y+scale/2;
  x += scale/2;
  
  y /= scale;
  x /= scale;
  
  if (mstate == GLUT_DOWN) {
    if (mbutton == GLUT_LEFT_BUTTON) {
      dstar->updateCell(x, y, -1);
    } else if (mbutton == GLUT_RIGHT_BUTTON) {
      dstar->updateStart(x, y);
    } else if (mbutton == GLUT_MIDDLE_BUTTON) {
      dstar->updateGoal(x, y);
    }
    replan = true;
  }
}

int main(int argc, char **argv) {

  glutInit(&argc, argv);  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  
  glutInitWindowSize(1000, 800);  
  glutInitWindowPosition(50, 20);  
    
  window = glutCreateWindow("Dstar Visualizer");  
  
  glutDisplayFunc(&DrawGLScene);  
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);
  glutMouseFunc(&mouseFunc);
  glutMotionFunc(&mouseMotionFunc);

  InitGL(800, 600);

  dstar = new dstar_lite::DStarLite();
  dstar->init(40,50,140, 90);
  
  printf("----------------------------------\n");
  printf("Dstar Visualizer\n");
  printf("Commands:\n");
  printf("[q/Q] - Quit\n");
  printf("[r/R] - Replan\n");
  printf("[a/A] - Toggle Auto Replan\n");
  printf("[c/C] - Clear (restart)\n");
  printf("[+/-] - Increments /  decrements the cost of the mouse-over-cell.\n");
  printf("[m/M] - Creates a random map around the start position filled with double values\n");
  printf("left mouse click - make cell untraversable (cost -1)\n");
  printf("middle mouse click - move goal to cell\n");
  printf("right mouse click - move start to cell\n");
  printf("----------------------------------\n");

  glutMainLoop();  

  return 1;
}
