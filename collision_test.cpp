
#include <iostream>
#include <vector>
#include "collision.h"
#include <cmath>

using namespace std;

class sphere: public collidable {
  double x, y, z, r;
  public:
  sphere(double nx, double ny, double nz, double nr) {
    x = nx;
    y = ny;
    z = nz;
    r = nr;
  }
  vec3 collision_point(vec3 dir) const{
    double l = dir.len();
    if (l == 0) {
      dir = vec3(0.0, 0.0, 0.0);
    }
    else {
      dir *= r/l;
    }
    return vec3(x, y, z) + dir;
  }
};

class cube: public collidable {
  vec3 c;
  vector<vec3> points;
  public:
  cube(double nx, double ny, double nz, double nr, double rx, double rz) : c(nx, ny, nz) {
    points.push_back(vec3(-1*nr, -1*nr, -1*nr));
    points.push_back(vec3(-1*nr, -1*nr,    nr));
    points.push_back(vec3(-1*nr,    nr,    nr));
    points.push_back(vec3(-1*nr,    nr, -1*nr));
    points.push_back(vec3(   nr,    nr, -1*nr));
    points.push_back(vec3(   nr,    nr,    nr));
    points.push_back(vec3(   nr, -1*nr,    nr));
    points.push_back(vec3(   nr, -1*nr, -1*nr));

    cout << "CUBE" << endl;
    for (vector<vec3>::iterator it = points.begin(); it != points.end(); it++) {
      double x = it->x,
             y = it->y,
             z = it->z;
      it->y = cos(rx)*y - sin(rx)*z;
      it->z = sin(rx)*y + cos(rx)*z;
      y = it->y;
      z = it->z;

      it->x = cos(rz)*x - sin(rz)*y;
      it->y = sin(rz)*x + cos(rz)*y;

      vec3 tmp = c + *it;
      cout << tmp << endl;
    }
    cout << endl;
  }
  vec3 collision_point(vec3 dir) const {
    double l = dir.len();
    if (l == 0) {
      dir = vec3(0.0, 0.0, 0.0);
    }
    else {
      vec3 max;
      double dist = 0;
      for (vector<vec3>::const_iterator it = points.begin(); it != points.end(); it++) {
        double n = ((*it) * dir).lenSq() / (*it).lenSq();
        if (n > dist) {
          dist = n;
          max = *it;
        }
      }
      dir = max;
    }
    return c + dir;
  }
};

void test(const collidable &a, const collidable &b) {
  cout << (collide(a, b) ? "HIT" : "NO HIT") << endl;
  cout << endl;
}

int main(int argc, char** argv) {

  collidable *a, *b;

  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.1);
  test(*a, *b);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 0.9);
  test(*a, *b);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.0);
  test(*a, *b);

/*
  a = new sphere(-1, 0, 0, 1);
  b = new cube(1.0, 0.0, 0.0, 1.5, 0.0, 0.0);
  test(*a, *b);
*/
}