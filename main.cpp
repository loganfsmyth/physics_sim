
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace std;

class Game {
  bool close;

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

    render(100*accum/step);
  }
}

Game::Game() {
  close = false;

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
  gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, 0.1, 100.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

}

void Game::simulate(unsigned long step) {


}

void Game::render(int interp_percent) {

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(-1.5f, 0.0f, -6.0f);
  glBegin(GL_TRIANGLES);
    glColor3f(1.0, 0.5, 0.5);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, 0.0f);
    glVertex3f(-1.0f, 1.0f, 0.0f);
  glEnd();
  SDL_GL_SwapBuffers();
}

void Game::check_events() {

  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_ACTIVEEVENT:
      case SDL_KEYDOWN:
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


