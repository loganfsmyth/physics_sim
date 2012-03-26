
#include <iostream>
#include <algorithm>
#include <list>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "game.h"

using namespace std;

const int mouseoffset = 200;

Game::Game() : updown(0), leftright(-180), width(800), height(600), close(false), FPS(60) {
  for (int i = 0; i < 4; i++) {
    movement[i] = false;
  }

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    throw exception();
  }
  SDL_WM_GrabInput(SDL_GRAB_ON);
  SDL_ShowCursor(0);

  // Set up the window and projection matrices for given window size.
  resize(width, height);

  glShadeModel(GL_SMOOTH);
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glClearColor(0,0,0,0);

  camera = new box(vec3(0,0,6), 0.25);
//  objs.push_back(camera);

  objs.push_back(new box(vec3(10, 5, 7), 1.25));
  objs.push_back(new box(vec3(0, -4, -2), 2));
//  objs.push_back(new box(vec3(2, 2,0), 1, 4, 1));
//  objs.push_back(new tetrahedron(vec3(5,0,4), 2));
  // new tetrahedron(vec3(1, 2.01, -1), 2);
  // new tetrahedron(vec3(0, 2.01, -1), 2);
  // new tetrahedron(vec3(0, 1, 1.7), 2);


//  objs.push_back(new box(vec3(), 2));
}
Game::~Game() {
  SDL_Quit();
}

/**
 * Run game loop. Using time accumulation method from gafferongames.
 */
void Game::run() {
  using namespace boost::posix_time;

  ptime start(microsec_clock::universal_time());
  unsigned long step = 10000; // 10 ms
  unsigned long accum = 0;

  while (!close) {
    ptime cur(microsec_clock::universal_time());
    long delta = (cur-start).total_microseconds();
    start = cur;

    // Accumulate time and execute fixed simulation steps based on available time.
    accum += delta;
    while (accum >= step) {
      simulate(step);
      accum -= step;
    }

    // Process keypresses and mouseclicks and such.
    check_events();

    // Output Framerate calculation
    cout << (1000000.0/delta) << "\r";
    render(100*accum/step);

    // Attempt to maintain a constant FPS, and avoid 100% CPU usage
    long us = 1000000.0/FPS - (microsec_clock::universal_time() - cur).total_microseconds();
    if (us > 1000) {
      boost::this_thread::sleep(microseconds(us));
    }

    // Keep mouse at fixed location to allow infinite movement
    SDL_WarpMouse(mouseoffset,mouseoffset);
  }
  cout << endl;
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

  // Find the object at the front.
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

  // Just set a bool for now. @TODO
  if (picked_name != 0) {
    objects[picked_name]->picked = true;
  }
}

/**
 * When the program initializes or the window is drag-resized, the window
 * context and projection matrices need to be updated with the new values.
 */
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

    // Rotate movement direction based on current xz-plane angle.
    double tmp = move.x*cos(rad) - move.z*sin(rad);
    move.z = move.x*sin(rad) + move.z*cos(rad);
    move.x = tmp;

    move *= 4.0 * step / 1000000;
    camera->st.pos += move;
    camera->st.mo = vec3();
  }

  // Calculate next position/orientation for all objects.
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->calcNext(step);
  }

  // Do simple 1-1 check for collision, since there are not that many items yet.
  // This will need to be sped up if there are tons of items colliding.
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    for (list<gameobj*>::iterator it2 = it; it2 != objs.end(); it2++) {
      if (*it == *it2 || *it == camera) continue;
      gameobj &a = **it;
      gameobj &b = **it2;
      list<vec3> a_pts, b_pts;
      vec3 sep;

      // Check if there is a collision, and react if so.
      if (contact_points(a, b, a_pts, b_pts, sep)) {
        triggerCollision(a, b, a_pts, sep);
      }
    }

    // Save calculated state for next round.
    // This could just be done in calcNext but I want to at some point be
    // able to better handle multi-collision circumstances.
    (*it)->commit();
  }
}

/**
 * Renders all the various objects in the scene.
 */
void Game::render(int interp_percent) {

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Simulate camera look
  glRotated(-1*updown, 1, 0, 0);
  glRotated(leftright, 0, 1, 0);

  // Move camera
  glTranslated(camera->st.pos.x, camera->st.pos.y, camera->st.pos.z);
  glColor3f(1.0f, 1.0f, 0.7f);

  // Render everything.
//  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  for (list<gameobj*>::iterator it = objs.begin(); it != objs.end(); it++) {
    (*it)->render();
  }
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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

        // Print out basic scene data when a key is pressed.
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

          /**
           * Log keypress status in custom array.
           */
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
            break;
          }
        }
        break;

      case SDL_MOUSEMOTION:
        // Process mouselook for camera.
        if (abs(e.motion.x-mouseoffset) < mouseoffset) { // limit x to box around 0,0
          leftright += (e.motion.x-mouseoffset) / 10.0;
          updown -= (e.motion.y-mouseoffset) / 10.0;
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        // Pick object on click
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