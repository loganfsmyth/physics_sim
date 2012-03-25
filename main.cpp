
#include <iostream>
#include <algorithm>
#include <list>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "collision.h"

using namespace std;

const int mouseoffset = 200;

class gameobj;
class Game {
  bool close;
  double leftright;
  double updown;
  vec3 position;
  list<gameobj*> objs;
  bool movement[4];
  vec3 sep;

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

  void rotate(double om, char axis = 'y');
};

vec3 calcCollisionVector(const gameobj &a, const gameobj &b) {
  
  return vec3();
}

void gameobj::rotate(double om, char axis) {
  const double a = om * 3.14159265358979 / 180;
  for (vector<vec3>::iterator it = pts.begin(); it != pts.end(); it++) {
    double x = it->x, y = it->y, z = it->z;
    switch (axis) {
      case 'x':
        it->y = y*cos(a) - z*sin(a);
        it->z = y*sin(a) + z*cos(a);
        break;
      case 'y':
        it->x = x*cos(a) - z*sin(a);
        it->z = x*sin(a) + z*cos(a);
        break;
      case 'z':
        it->x = x*cos(a) - y*sin(a);
        it->y = x*sin(a) + y*cos(a);
        break;
      default:
        break;
    }
  }
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
    double c = 0.2;
    glTranslated(pos.x, pos.y, pos.z);
    glBegin(GL_QUADS);
    for (vector<int>::const_iterator it = index.begin(); it != index.end();) {
      glColor3d(c, c+0.1, c+0.2);
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      c += 0.1;
    }
    glEnd();

    glTranslated(-1*pos.x, -1*pos.y, -1*pos.z);
    
    
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3d(1.0, 0, 0);
    for (vector<vec3>::const_iterator it = sim_pts.begin(); it != sim_pts.end(); it++) {
      const vec3 &v = *it;
      glVertex3d(it->x, it->y, it->z);
    }
    glEnd();
    
    glBegin(GL_LINES);
    glColor3d(0, 1.0, 0);
    for (vector<pair<vec3,vec3> >::const_iterator it = sim_edges.begin(); it != sim_edges.end(); it++) {
      const pair<vec3,vec3> &v = *it;
      const vec3 f = v.first + vec3(0.05, 0, 0);
      const vec3 s = v.second + vec3(0.05, 0, 0);
      glVertex3d(f.x, f.y, f.z);
      glVertex3d(s.x, s.y, s.z);
    }
    glEnd();
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
    double c = 0.5;
    glBegin(GL_TRIANGLES);
    for (vector<int>::const_iterator it = index.begin(); it != index.end();) {
      glColor3d(c, c+0.1, c);
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
      c += 0.1;
    }
    glEnd();

    glTranslated(-1*pos.x, -1*pos.y, -1*pos.z);
    
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3d(1.0, 0, 0);
    for (vector<vec3>::const_iterator it = sim_pts.begin(); it != sim_pts.end(); it++) {
      const vec3 &v = *it;
      glVertex3d(it->x, it->y, it->z);
    }
    glEnd();

  }


};




void ud(collidable* a, collidable* b, vec3 &sep) {
  a->sim_pts.clear();
  a->sim_edges.clear();
  b->sim_pts.clear();
  b->sim_edges.clear();

  list<vec3> a_pts, b_pts, contact;
  if (contact_points(*a, *b, a_pts, b_pts, sep)) {
    for (list<vec3>::iterator it = a_pts.begin(); it != a_pts.end(); it++) {
      a->sim_pts.push_back(*it);
    }
    for (list<vec3>::iterator it = b_pts.begin(); it != b_pts.end(); it++) {
      b->sim_pts.push_back(*it);
    }
  }
  else {
    cout << "NOT COLLIDING" << endl;
  }
}


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

    SDL_WarpMouse(mouseoffset,mouseoffset);
  }
  cout << endl;
}

Game::Game() : updown(0), leftright(-180), position(0,0, 6) {
  close = false;

  int w = 800,
      h = 600;
  for (int i = 0; i < 4; i++) {
    movement[i] = false;
  }

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    throw exception();
  }
  SDL_WM_GrabInput(SDL_GRAB_ON);
  SDL_ShowCursor(0);

  resize(w, h);

  glShadeModel(GL_SMOOTH);
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glClearColor(0,0,0,0);
/*
  gameobj *a = new tetrahedron(vec3(), 2),
          //*b = new tetrahedron(vec3(1, 2.01, -1), 2);
          //*b = new tetrahedron(vec3(0, 2.01, -1), 2);
          *b = new tetrahedron(vec3(0, 1, 1.7), 2);
*/
  gameobj *a = new box(vec3(), 2),
          *b = new box(vec3(2, 2,0), 1, 4, 1);
          //*b = new box(vec3(0, 2,0), 1, 4, 1);

  b->rotate(180, 'y');
  for (vector<vec3>::iterator it = b->pts.begin(); it != b->pts.end(); it++) {
    vec3 v = *it + b->pos;
    cout << v;
  }
  cout << endl;


/*
  vec3 v(a->collision_point(vec3(0,1,0)));
  cout << v << endl;
  v = a->collision_point(vec3(0, 0, -1));
  cout << v << endl;

  v = a->collision_point(vec3(1, 0, 1));
  cout << v << endl;

  v = a->collision_point(vec3(-1, 0, 1));
  cout << v << endl;


  v = b->collision_point(vec3(0,1,0));
  cout << v << endl;
  v = b->collision_point(vec3(0, 0, -1));
  cout << v << endl;

  v = b->collision_point(vec3(1, 0, 1));
  cout << v << endl;

  v = b->collision_point(vec3(-1, 0, 1));
  cout << v << endl;
*/
  objs.push_back(a);
  objs.push_back(b);
  
  vec3 pa, pb, da, db;
  //sep = collision_point(*a, *b, pa, pb, da, db);

  ud(a, b, sep);

  cout << a->sim_pts.size() << " = " << b->sim_pts.size() << endl;

  cout << sep << "-" << pa << pb << "=" << da << db << endl;

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

  vec3 move;
  const double rad = leftright*3.14159265/180;
  if (movement[0]) move.z += 1; // up
  if (movement[1]) move.z -= 1; // down
  if (movement[2]) move.x += 1; // left
  if (movement[3]) move.x -= 1; // right
  move.norm();

  double tmp = move.x*cos(rad) - move.z*sin(rad);
  move.z = move.x*sin(rad) + move.z*cos(rad);
  move.x = tmp;

  move *= 4.0 * step / 1000000;
  position += move;

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

  glRotated(-1*updown, 1, 0, 0);
  glRotated(leftright, 0, 1, 0);

  glTranslated(position.x, position.y, position.z);
  glColor3f(1.0f, 1.0f, 0.7f);

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->render();
  }
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  vec3 v = sep * 1000;
  
  

  glLineWidth(2);
  glBegin(GL_LINES);
    glColor3d(1.0, 0, 0);
    glVertex3d(-v.x, -v.y, -v.z);
    glVertex3d( v.x,  v.y,  v.z);
  glEnd();

  glFinish();
  SDL_GL_SwapBuffers();
}


void Game::check_events() {
  double step = 0.1;

  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_ACTIVEEVENT:
        break;
      case SDL_KEYDOWN:
        cout << "UpDown: " << updown << " Leftright:" << leftright << " pos:" << position << endl;

        switch (e.key.keysym.sym) {
          case SDLK_ESCAPE:
            close = true;
            break;
          case SDLK_UP:
            movement[0] = true;
            break;
          case SDLK_DOWN:
            movement[1] = true;
            break;
          case SDLK_LEFT:
            movement[2] = true;
            break;
          case SDLK_RIGHT:
            movement[3] = true;
            break;
          case SDLK_e:
            (*(++objs.begin()))->pos += vec3(0, step, 0);
            break;
          case SDLK_q:
            (*(++objs.begin()))->pos -= vec3(0, step, 0);
            break;
          case SDLK_w:
            (*(++objs.begin()))->pos += vec3(step, 0, 0);
            break;
          case SDLK_s:
            (*(++objs.begin()))->pos -= vec3(step, 0, 0);
            break;
          case SDLK_a:
            (*(++objs.begin()))->pos += vec3(0, 0, step);
            break;
          case SDLK_d:
            (*(++objs.begin()))->pos -= vec3(0, 0, step);
            break;
          case SDLK_1:
            (*(++objs.begin()))->rotate(15, 'x');
            break;
          case SDLK_2:
            (*(++objs.begin()))->rotate(-15, 'x');
            break;
          case SDLK_3:
            (*(++objs.begin()))->rotate(15, 'y');
            break;
          case SDLK_4:
            (*(++objs.begin()))->rotate(-15, 'y');
            break;
          default:
            break;
        }
        break;
      case SDL_KEYUP:
        switch (e.key.keysym.sym) {
          case SDLK_UP:
            movement[0] = false;
            break;
          case SDLK_DOWN:
            movement[1] = false;
            break;
          case SDLK_LEFT:
            movement[2] = false;
            break;
          case SDLK_RIGHT:
            movement[3] = false;
            break;

          default: {
            collidable *a = *objs.begin();
            collidable *b = *++objs.begin();
            vec3 pa, pb, da, db;

            ud(a, b, sep);
            break;
          }
        }
        break;

      case SDL_MOUSEMOTION:
        if (abs(e.motion.x-mouseoffset) < mouseoffset) { // limit x to box around 0,0
          leftright += (e.motion.x-mouseoffset) / 10.0;
          updown -= (e.motion.y-mouseoffset) / 10.0;
        }
        break;
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


