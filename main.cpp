
#include <iostream>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace std;

class Game {
  bool close;

  float angle;

  public:
  Game();
  ~Game();
  void run();
  void simulate(unsigned long step);
  void render(int interp_percent);
  void check_events();

  void resize(int h, int w);
};


void Game::run() {
  using namespace boost::posix_time;

  ptime start(microsec_clock::universal_time());
  unsigned long step = 10000; // 10 ms
  unsigned long accum = 0;

  while (!close) {
    ptime cur(microsec_clock::universal_time());
    long delta = (cur-start).total_microseconds();
    start = cur;

    accum += delta;
    while (accum >= step) {
      simulate(step);

      check_events();
      accum -= step;
    }

    for (int i = 0; i < 10000; i++);

    cout << (1000000.0/delta) << "\r";
    render(100*accum/step);
  }
  cout << endl;
}

Game::Game() {
  close = false;
  angle = 0.0f;

  int w = 640,
      h = 480;

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    throw exception();
  }

  resize(w, h);

  glShadeModel(GL_SMOOTH);
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glClearColor(0,0,0,0);
}
Game::~Game() {

  SDL_Quit();
}

void Game::resize(int w, int h) {

  if (h == 0) h = 1;

  if (SDL_SetVideoMode(w, h, 32, SDL_HWSURFACE|SDL_GL_DOUBLEBUFFER|SDL_OPENGL|SDL_RESIZABLE) == NULL) {
    throw exception();
  }
  glViewport(0,0,w,h);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, 0.1, 2000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

}

void Game::simulate(unsigned long step) {


}

void plane(GLfloat w, GLfloat h, int axis = 0) {
  // axis: xy:0, yz:1, xz:2

  GLfloat pts[4][3];
  GLfloat* start = (GLfloat*)pts;
  fill(start, start+4*3, (GLfloat)0.0f);
  GLfloat x1 = -1*w/2,
          x2 = w/2,
          y1 = -1*h/2,
          y2 = h/2;


  switch (axis) {
    case 0:
      pts[0][0] = x1; pts[0][1] = y1;
      pts[1][0] = x1; pts[1][1] = y2;
      pts[2][0] = x2; pts[2][1] = y2;
      pts[3][0] = x2; pts[3][1] = y1;
      break;
    case 1:
      pts[0][1] = x1; pts[0][2] = y1;
      pts[1][1] = x1; pts[1][2] = y2;
      pts[2][1] = x2; pts[2][2] = y2;
      pts[3][1] = x2; pts[3][2] = y1;
      break;
    case 2:

      pts[0][0] = x1; pts[0][2] = y1;
      pts[1][0] = x1; pts[1][2] = y2;
      pts[2][0] = x2; pts[2][2] = y2;
      pts[3][0] = x2; pts[3][2] = y1;
      break;
  }

  glBegin(GL_QUADS);
  for (int i = 0; i < 4; i++) {
    glColor3f(pts[i][0], pts[i][1], pts[i][2]);
    glVertex3f(pts[i][0], pts[i][1], pts[i][2]);
  }
  glEnd();
}

void box(GLfloat h, GLfloat w, GLfloat l) {
  glPushMatrix();

  glTranslatef(0.0f, 0.0f, -1*l/2);
  plane(w, h);
  glTranslatef(0.0f, 0.0f, l);
  plane(w, h);

  glTranslatef(-1*w/2, 0.0f, -1*l/2);
  plane(l, h, 1);
  glTranslatef(w, 0.0f, 0.0f);
  plane(l, h, 1);

  glTranslatef(-1*w/2, -1*h/2, 0.0f);
  plane(l, w, 2);
  glTranslatef(0.0f, h, 0.0f);
  plane(l, w, 2);

  glPopMatrix();
}


void Game::render(int interp_percent) {

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();


  glTranslatef(0.0f, -1.0f, -6.0f);
  glColor3f(1.0f, 1.0f, 0.7f);
  plane(20.0f, 20.0f, 2);

  glRotatef(angle, 0.0f, 1.0f, 0.0f);
  box(1.0f, 1.0f, 1.0f);

  SDL_GL_SwapBuffers();
}


void Game::check_events() {

  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_ACTIVEEVENT:
        break;
      case SDL_KEYDOWN:
        angle += 4;
        break;
      case SDL_KEYUP:

      case SDL_MOUSEMOTION:
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
        break;
      case SDL_QUIT:
        close = true;
        break;
      case SDL_SYSWMEVENT:
      case SDL_VIDEORESIZE:
        resize(e.resize.w, e.resize.h);
        break;
      case SDL_VIDEOEXPOSE:
      default:
        break;
    }
  }

}

int main(int argc, char** argv) {

  Game g;
  g.run();
  return 0;
}


