
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <cmath>
#include "gameobj.h"
#include <iostream>

using namespace std;

Quaternion Quaternion::operator*(double d) const {
  Quaternion q(*this);
  q.w *= d;
  q.x *= d;
  q.y *= d;
  q.z *= d;
  return q;
}
Quaternion Quaternion::operator*(const Quaternion &q) const {
  Quaternion out;
  out.w = w*q.x + x*q.w + y*q.z - z*q.y;
  out.x = w*q.y + y*q.w + z*q.x - x*q.z;
  out.y = w*q.z + z*q.w + x*q.y - y*q.x;
  out.z = w*q.w - x*q.x - y*q.y - z*q.z;
  return out;
}
vec3 Quaternion::operator*(vec3 v) const {
  double l = v.len();
  v.norm();
  Quaternion q;
  q.x = v.x;
  q.y = v.y;
  q.z = v.z;
  q.w = 0;

  Quaternion ret = q * conj();
  ret = *this * ret;

  vec3 rv(ret.x, ret.y, ret.z);
  rv.norm();
  rv *= l;
  return rv;
}
Quaternion& Quaternion::operator+=(const Quaternion q) {
  w += q.w;
  x += q.x;
  y += q.y;
  z += q.z;
  return *this;
}
Quaternion Quaternion::operator+(const Quaternion q1) const {
  Quaternion q(*this);
  q.w += q1.w;
  q.x += q1.x;
  q.y += q1.y;
  q.z += q1.z;
  return q;
}
void Quaternion::norm() {
  double len = 1.0/sqrt(w*w + x*x + y*y + z*z);
  w *= len;
  x *= len;
  y *= len;
  z *= len;
}
Quaternion Quaternion::conj() const {
  Quaternion q(*this);
  q.x *= -1;
  q.y *= -1;
  q.z *= -1;
  return q;
}

void Quaternion::fromAxisAngle(vec3 v, double angle) {
  angle /= 2;
  v.norm();
  double s = sin(angle);
  x = v.x * s;
  y = v.y * s;
  z = v.z * s;
  w = cos(angle);
}

void Quaternion::axisAngle(vec3 &axis, double &angle) const {
  double scale = sqrt(x*x + y*y + z*z);
  if (scale == 0) {
    axis = vec3();
  }
  else {
    axis.x = x / scale;
    axis.y = y / scale;
    axis.z = z / scale;
  }
  angle = acos(w) / 2.0;
}

void State::recalc() {
  invMass = 1.0/mass;
  invInertiaTensor = 1.0/inertia;
  vel = mo * invMass;
  angVel = angMo * invInertiaTensor;

  orient.norm();
  Quaternion q;
  q.w = 0;
  q.x = angVel.x;
  q.y = angVel.y;
  q.z = angVel.z;
  spin = q * orient * 0.5;

/** /
  cout << "Recalc:" << endl;
  cout << "Mass:" << mass << endl;
  cout << "Vel: " << vel << endl;
  cout << "Mo: " << mo << endl;

  cout << "AngVel: " << angVel << endl;
  cout << "AngMo:" << angMo << endl;
/**/

}

Derivative evalDer(State s) {
  Derivative d;
  d.dv = s.vel;
  d.df = s.pos * -10 - s.mo;
  d.spin = s.spin;
  d.torque -= s.angMo * (0.1 / s.inertia);
  return d;
}

Derivative evalDer(State s, double ms, Derivative &d) {
  s.pos += d.dv * ms;
  s.mo += d.df * ms;
  s.orient += d.spin * ms;
  s.angMo += d.torque * ms;

  s.recalc();
  
  Derivative out;
  out.dv = s.mo;
  out.df = s.pos * -10 - s.mo;
  out.spin = d.spin;
  out.torque -= s.angMo * (0.1 / s.inertia);
  return out;
}


void triggerCollision(gameobj& a, gameobj &b, list<vec3> pts, vec3 &n) {
  
  a.next_st.mo -= n*(2*a.next_st.mo.dot(n));
  b.next_st.mo -= n*(2*b.next_st.mo.dot(n));

}

gameobj::gameobj(vec3 c) : picked(false) {
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

  Derivative a = evalDer(st);
  Derivative b = evalDer(st, step/2, a);
  Derivative c = evalDer(st, step/2, b);
  Derivative d = evalDer(st, step, c);

  const vec3 dvdt = (a.dv + (b.dv + c.dv)*2 + d.dv) * (1/6.0);
  const vec3 dfdt = (a.df + (b.df + c.df)*2 + d.df) * (1/6.0);
  const Quaternion spin = (a.spin + (b.spin + c.spin)*2 + d.spin) * (1/6.0);
  const vec3 torque = (a.torque + (b.torque + c.torque)*2 + d.torque) * (1/6.0);

//  cout << "(" << spin.w << ',' << spin.x << ',' << spin.y << ',' << spin.z << ')' << endl;

  next_st = st;
  next_st.pos = st.pos + dvdt*step;
  next_st.mo = st.mo + dfdt*step;
  next_st.orient = st.orient + spin * step;
  next_st.angMo = st.angMo + torque * step;

  Quaternion q;
  q.fromAxisAngle(vec3(1,0,0), 0.0001);
//  next_st.orient = next_st.orient * q;

  next_st.recalc();

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
  st.orient.w = 1;
  st.mass = 1;
  st.inertia = st.mass * w*w / 6.0;
  st.recalc();

  Quaternion q;
  q.fromAxisAngle(vec3(1,0,0), 1.5);

  cout << q.w << " " << q.x << " " << q.y << " " << q.z << endl;

  vec3 v(0,1,0);

  vec3 n = q*v;
  cout << n << endl;


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

void box::render(bool pick) const {
  int i = 0;
  double c = 0.2;
  
  vec3 axis;
  double angle;
  st.orient.axisAngle(axis, angle);

  glTranslated(st.pos.x, st.pos.y, st.pos.z);
  glRotated(angle * 180 / 3.1415926589, axis.x, axis.y, axis.z);
  
  glBegin(GL_QUADS);
  for (vector<int>::const_iterator it = index.begin(); it != index.end();) {
    glColor3d(c, c+0.1, c+0.2);

    if (picked) {
      glColor3d(0, 1, 0);
    }

    glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    glVertex3d(pts[*it].x, pts[*it].y, pts[*it].z); it++;
    c += 0.1;
  }
  glEnd();

  glRotated(-angle, axis.x, axis.y, axis.z);
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
void tetrahedron::render(bool pick) const {
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


