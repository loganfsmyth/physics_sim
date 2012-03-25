
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <cmath>
#include "gameobj.h"
#include <iostream>

using namespace std;


vec3 acceleration(const State &s) {
  const float k = 10.0;
  const float b = 1;
  return (s.pos * -k) - (s.mo * b);
}

Derivative evalDer(State s, int ms, Derivative &d) {
  s.pos += d.dv * ms;
  s.mo += d.df * ms;

  Derivative out;
  out.dv = s.mo;
  out.df = acceleration(s);
  return out;
}


void triggerCollision(gameobj& a, gameobj &b, list<vec3> pts, vec3 &n) {
  
  a.next_st.mo -= n*(2*a.st.mo.dot(n));
  b.next_st.mo -= n*(2*b.st.mo.dot(n));

  //vec3 mo = a.st.mo + b.st.mo;
  
}

gameobj::gameobj(vec3 c) {
  st.pos = c;

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
void gameobj::calcNext(unsigned long ms) {
  double step = ms / 1000000.0;

  

  Derivative start;
  Derivative a = evalDer(st, 0.0, start);
  Derivative b = evalDer(st, step/2, a);
  Derivative c = evalDer(st, step/2, b);
  Derivative d = evalDer(st, step, c);

  const vec3 dvdt = (a.dv + (b.dv + c.dv)*2 + d.dv) * (1/6.0);
  const vec3 dfdt = (a.df + (b.df + c.df)*2 + d.df) * (1/6.0);

//  cout << "DV" << dvdt << dfdt << endl;

  next_st.pos = st.pos + dvdt*step;
  next_st.mo = st.mo + dfdt*step;
}
void gameobj::commit() {
  st = next_st;
}

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
  return max + st.pos;
}





void box::init(vec3 c, double w, double h, double l) {
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
}

box::box(vec3 c, double w, double h, double l): gameobj(c) {
  init(c, w, h, l);
}
box::  box(vec3 c, double w) : gameobj(c) {
  init(c, w, w, w);
}

void box::render() const {
  int i = 0;
  double c = 0.2;
  glTranslated(st.pos.x, st.pos.y, st.pos.z);
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

  glTranslated(-1*st.pos.x, -1*st.pos.y, -1*st.pos.z);
  
  
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

void tetrahedron::init(double w) {
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

tetrahedron::tetrahedron(vec3 c, double w): gameobj(c) {
  init(w);
}
void tetrahedron::render() const {
  int i = 0;
  glTranslated(st.pos.x, st.pos.y, st.pos.z);
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

  glTranslated(-1*st.pos.x, -1*st.pos.y, -1*st.pos.z);
  
  glPointSize(10);
  glBegin(GL_POINTS);
  glColor3d(1.0, 0, 0);
  for (vector<vec3>::const_iterator it = sim_pts.begin(); it != sim_pts.end(); it++) {
    const vec3 &v = *it;
    glVertex3d(it->x, it->y, it->z);
  }
  glEnd();

}


