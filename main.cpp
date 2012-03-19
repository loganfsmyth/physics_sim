
#include <iostream>
#include <algorithm>
#include <list>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "collision.h"

using namespace std;


class gameobj;
class Game {
  bool close;
  float angle;
  list<gameobj*> objs;

  public:
  Game();
  ~Game();
  void run();
  void simulate(unsigned long step);
  void render(int interp_percent);
  void check_events();

  void resize(int h, int w);
};

class gameobj: public collidable {
  public:
  vector<vec3> pts;
  vector<int> index;
  vector<vec3> normals;
  int vert_per_poly;
  vec3 pos;

  public:
  gameobj(vec3 c): pos(c) { }
  virtual void calcNext(unsigned long step);
  virtual void commit();
  virtual void triggerCollision(vec3 normal);
  virtual vec3 collision_point(vec3 dir) const;
  virtual void render() const = 0;

};

vec3 calcCollisionVector(const gameobj &a, const gameobj &b) {
  
  return vec3();
}

void gameobj::calcNext(unsigned long step) { }
void gameobj::commit() { }
void gameobj::triggerCollision(vec3 norm) { }
vec3 gameobj::collision_point(vec3 dir) const {
  double angle = 0;
  vec3 max;

  vector<vec3>::const_iterator it;

  vec3 centroid;
  for (it = pts.begin(); it != pts.end(); it++) {
    centroid += *it;
  }
  centroid *= 1/pts.size();

  for (it = pts.begin(); it != pts.end(); it++) {
    vec3 pt = *it-centroid;
    double a = pt.dot(dir)/(pt.len()*dir.len());
    if (a > angle) {
      angle = a;
      max = *it;
    }
  }
  return max + pos;
}


class box : public gameobj {

  void init(vec3 c, double w, double h, double l) {
    vert_per_poly = 4;
    pts.reserve(8);
    pts.push_back(vec3(-w/2, -h/2, -l/2));
    pts.push_back(vec3( w/2, -h/2, -l/2));
    pts.push_back(vec3(-w/2,  h/2, -l/2));
    pts.push_back(vec3( w/2,  h/2, -l/2));
    pts.push_back(vec3(-w/2, -h/2, l/2));
    pts.push_back(vec3( w/2, -h/2, l/2));
    pts.push_back(vec3(-w/2,  h/2, l/2));
    pts.push_back(vec3( w/2,  h/2, l/2));

    index.reserve(6*4);
    index.push_back(0); index.push_back(2); index.push_back(3); index.push_back(1); // back face
    index.push_back(0); index.push_back(4); index.push_back(6); index.push_back(2); // left side
    index.push_back(4); index.push_back(5); index.push_back(7); index.push_back(6); // front face
    index.push_back(1); index.push_back(3); index.push_back(7); index.push_back(5); // right face
    index.push_back(2); index.push_back(6); index.push_back(7); index.push_back(3); // top face
    index.push_back(0); index.push_back(1); index.push_back(5); index.push_back(4); // bottom face
/**/
  }

  public:
  box(vec3 c, double w, double h, double l): gameobj(c) {
    init(c, w, h, l);
  }
  box(vec3 c, double w) : gameobj(c) {
    init(c, w, w, w);
  }

  virtual void render() const {
    int i = 0;
    glTranslated(pos.x, pos.y, pos.z);
    glBegin(GL_QUADS);
    for (vector<int>::const_iterator it = index.begin(); it != index.end();) {
      double sum = (pts[*it].x + pts[*it].y + pts[*it].z) / 8;
      if (sum < 0) sum *= -1;
      glColor3d(sum, sum, sum);
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    }
    glEnd();
    glTranslated(-1*pos.x, -1*pos.y, -1*pos.z);
  }
};

class tetrahedron: public gameobj {
  void init(double w) {
    vert_per_poly = 3;
    pts.reserve(4);
    pts.push_back(vec3(   0, w/2, 0));
    pts.push_back(vec3(   0,-w/2,-w/2));
    pts.push_back(vec3(-w/2,-w/2, w/2));
    pts.push_back(vec3( w/2,-w/2, w/2));

    index.reserve(4*3);
    index.push_back(0); index.push_back(2); index.push_back(3);
    index.push_back(0); index.push_back(3); index.push_back(1);
    index.push_back(0); index.push_back(1); index.push_back(2);
    index.push_back(3); index.push_back(2); index.push_back(1);
  }

  public:
  tetrahedron(vec3 c, double w): gameobj(c) {
    init(w);
  }
  virtual void render() const {
    int i = 0;
    glTranslated(pos.x, pos.y, pos.z);
    glBegin(GL_TRIANGLES);
    for (vector<int>::const_iterator it = index.begin(); it != index.end();) {
      double sum = (pts[*it].x + pts[*it].y + pts[*it].z) / 8;
      if (sum < 0) sum *= -1;
      glColor3d(sum, sum, sum);
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    }
    glEnd();
    glTranslated(-1*pos.x, -1*pos.y, -1*pos.z);
  }

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

//  objs.push_back(new box(vec3(), 2, 3, 4));
  objs.push_back(new tetrahedron(vec3(), 1));
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







  return;
  while (step) {
    for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
      (*it)->calcNext(step);
    }
    list<pair<gameobj*,gameobj*> > had_collision;
    for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
      for (list<gameobj*>::iterator it2 = objs.begin(); it2 != objs.end(); it++) {
        if (collide(**it, **it2)) {
          had_collision.push_back(pair<gameobj*,gameobj*>(*it, *it2));
        }
      }
    }
    if (had_collision.size()) {
      int min = 0, max = step, mid;
      list<pair<gameobj*,gameobj*> >::iterator hit_it;
      list<gameobj*>::iterator it;
      while ( min <= max) {
        mid = (max+min)/2;
        for (it = objs.begin(); it != objs.end(); it++) {
          (*it)->calcNext(mid);
        }
        list<pair<gameobj*,gameobj*> > new_had_collision;
        bool hit = false;
        for (hit_it = had_collision.begin(); hit_it != had_collision.end(); hit_it++) {
          if (collide(*(hit_it->first), *(hit_it->second))) {
            hit = true;
            new_had_collision.push_back(*hit_it);
          }
        }
        if (hit) {
          max = mid - 1;
          had_collision = new_had_collision;
        }
        else {
          min = mid + 1;
        }
      }
      if (max > 0) {
        for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
          (*it)->calcNext(max);
          (*it)->commit();
        }
        for (hit_it = had_collision.begin(); hit_it != had_collision.end(); hit_it++) {
          gameobj *f = hit_it->first, *s = hit_it->second;
          vec3 v = calcCollisionVector(*f, *s);
          f->triggerCollision(v);
          s->triggerCollision(v * -1);
        }
        step -= max;
      }
      else {
        step -= step;
      }
    }
    else {
      step -= step;
    }
  }
}

void Game::render(int interp_percent) {

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();


  glTranslatef(0.0f, -1.0f, -6.0f);
  glColor3f(1.0f, 1.0f, 0.7f);

  glRotatef(angle, 0.0f, 1.0f, 0.0f);

  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->render();
  }

  SDL_GL_SwapBuffers();
}


void Game::check_events() {

  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_ACTIVEEVENT:
        break;
      case SDL_KEYDOWN:
        angle += 14;
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


