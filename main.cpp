
#include <iostream>
#include <algorithm>
#include <list>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>


#include "gameobj.h"

using namespace std;

const int mouseoffset = 200;

class Game {
  bool close;
  double leftright;
  double updown;
  gameobj* camera;
  list<gameobj*> objs;
  bool movement[4];
  vec3 sep;
  int width, height;

  public:
  Game();
  ~Game();
  void run();
  void simulate(unsigned long step);
  void render(int interp_percent);
  void check_events();

  void resize(int h, int w);
  void pick(int x, int y);
};

vec3 calcCollisionVector(const gameobj &a, const gameobj &b) {
  
  return vec3();
}



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

Game::Game() : updown(0), leftright(-180) {
  close = false;

  width = 800,
  height = 600;
  for (int i = 0; i < 4; i++) {
    movement[i] = false;
  }

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    throw exception();
  }
  SDL_WM_GrabInput(SDL_GRAB_ON);
  SDL_ShowCursor(0);

  resize(width, height);

  glShadeModel(GL_SMOOTH);
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glClearColor(0,0,0,0);

  camera = new box(vec3(0,0,6), 0.25);
  objs.push_back(camera);
//  camera->physics = false;

  objs.push_back(new box(vec3(10, 5, 7), 1.25));
  objs.push_back(new box(vec3(0, -4, -2), 2));
  objs.push_back(new box(vec3(2, 2,0), 1, 4, 1));
  objs.push_back(new tetrahedron(vec3(5,0,4), 2));
  // new tetrahedron(vec3(1, 2.01, -1), 2);
  // new tetrahedron(vec3(0, 2.01, -1), 2);
  // new tetrahedron(vec3(0, 1, 1.7), 2);

  return;

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
    vec3 v = *it + b->st.pos;
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

//  ud(a, b, sep);

  cout << a->sim_pts.size() << " = " << b->sim_pts.size() << endl;

  cout << sep << "-" << pa << pb << "=" << da << db << endl;

}
Game::~Game() {

  SDL_Quit();
}

void Game::pick(int x, int y) {
  cout << x << " " << y << endl;

  const int pickBufferSize = 32;
  GLuint pickBuffer[pickBufferSize];
  GLint view[4];

  // Tell GL about the buffer and get view data
  glSelectBuffer(pickBufferSize, pickBuffer);
  glGetIntegerv(GL_VIEWPORT, view);

  // Start pick mode
  glRenderMode(GL_SELECT);
  glInitNames();
  glPushName(0);
  
  // Set up Matrices for 1px view at x,y coords
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluPickMatrix(x, y, 1.0, 1.0, view);
  gluPerspective(45.0f, (float)view[2] / (float)view[3], 0.1, 2000.0f);
  glMatrixMode(GL_MODELVIEW);

  // Draw items
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glRotated(-1*updown, 1, 0, 0);
  glRotated(leftright, 0, 1, 0);

  glTranslated(camera->st.pos.x, camera->st.pos.y, camera->st.pos.z);
  glColor3f(1.0f, 1.0f, 0.7f);

  map<GLubyte,gameobj*> objects;
  GLubyte i = 1;
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++, i++) {
    // Render each item with a name, and save name->obj mapping
    (*it)->picked = false;
    glLoadName(i);
    objects.insert(pair<GLubyte,gameobj*>(i, *it));
    (*it)->render(true);
  }

  // Revert projection matrix to normal
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();


  GLint hits = glRenderMode(GL_RENDER);

  glMatrixMode(GL_MODELVIEW);

  GLubyte min_z = 255;
  GLubyte picked_name = 0;
  for (int i = 0; i < hits; i += 4) {
    GLubyte num = pickBuffer[i],
            min = pickBuffer[i+1],
            max = pickBuffer[i+2],
            name = pickBuffer[i+3];

    if (min < min_z) {
      min_z = min;
      picked_name = name;
    }
  }

  if (picked_name != 0) {
    objects[picked_name]->picked = true;
  }
}

void Game::resize(int w, int h) {

  if (h == 0) h = 1;

  width = w;
  height = h;

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

  { // Handle camera movement
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
    camera->st.pos += move;
    camera->st.mo = vec3();
  }


  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->calcNext(step);
  }
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    for (list<gameobj*>::iterator it2 = it; it2 != objs.end(); it2++) {
      if (*it == *it2 || *it == camera) continue;
      gameobj &a = **it;
      gameobj &b = **it2;
      list<vec3> a_pts, b_pts;
      if (contact_points(a, b, a_pts, b_pts, sep)) {
        triggerCollision(a, b, a_pts, sep);
      }
    }
    (*it)->commit();
  }
}

void Game::render(int interp_percent) {

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glRotated(-1*updown, 1, 0, 0);
  glRotated(leftright, 0, 1, 0);

  glTranslated(camera->st.pos.x, camera->st.pos.y, camera->st.pos.z);
  glColor3f(1.0f, 1.0f, 0.7f);

//  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->render();
  }
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
        cout << "UpDown: " << updown << " Leftright:" << leftright << " pos:" << camera->st.pos << endl;

        for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
          cout << "Obj:" << (*it)->st.pos << endl;
          for (vector<vec3>::iterator it2 = (*it)->pts.begin(); it2 != (*it)->pts.end(); it2++) {
            cout << *it2 << endl;
          }
        }


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

/*
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
*/
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

//            ud(a, b, sep);
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
        pick(width/2, height/2);
        break;
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


