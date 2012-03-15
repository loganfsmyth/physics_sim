
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
  vec3 collision_point(vec3 dir) const {
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

class point: public collidable {
  vec3 p;
  public:
  point(double x, double y, double z): p(x,y,z) { }
  vec3 collision_point(vec3 dir) const {
    return p;
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

    cerr << "CUBE" << endl;
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
      cerr << tmp << endl;
    }
    cerr << endl;
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
        //double n = ((*it) * dir).lenSq() / (*it).lenSq();
        double n = it->dot(dir);
//        vec3 v = *it;
//        cerr << v << " && " << v.dot(dir) << endl;
        if (n > dist) {
//          cerr << "Up " << v << endl;
          dist = n;
          max = *it;
        }
      }

      cerr << "Max: " << max << endl;
      dir = max;
    }

    return c + dir;
  }
};

void test(const collidable &a, const collidable &b, bool ex) {
  bool hit = collide(a, b);
  cout << (hit == ex ? "PASS: " : "FAIL: ") << (hit ? "HIT" : "NO HIT") << endl;
  cerr << endl;
}

int main(int argc, char** argv) {

  collidable *a, *b;

/*
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.1);
  test(*a, *b, true);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 0.9);
  test(*a, *b, false);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.0);
  test(*a, *b, true);
/**/

//*

  cout << "Beyond faces of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 0.0, 0.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 0.0, 0.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 2.0, 0.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -2.0, 0.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.0, 2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.0, -2.0);
  test(*a, *b, false);
  

  cout << "Beyond corners of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 2.0, 2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 2.0, -2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, -2.0, 2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, -2.0, -2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 2.0, 2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 2.0, -2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, -2.0, 2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, -2.0, -2.0);
  test(*a, *b, false);

  cout << "Beyond edges of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 2.0, 2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 2.0, -2.0);
  test(*a, *b, false);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -2.0, 2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -2.0, -2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 0.0, 2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 0.0, -2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 0.0, 2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 0.0, -2.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, 2.0, 0.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(2.0, -2.0, 0.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, 2.0, 0.0);
  test(*a, *b, false);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-2.0, -2.0, 0.0);
  test(*a, *b, false);


  cout << "Inside faces of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.0, 0.0);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.0, 0.0);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.95, 0.0);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -0.95, 0.0);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.0, 0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.0, -0.95);
  test(*a, *b, true);
  

  cout << "Inside corners of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.95, 0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.95, -0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, -0.95, 0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, -0.95, -0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.95, 0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.95, -0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, -0.95, 0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, -0.95, -0.95);
  test(*a, *b, true);

  cout << "Inside edges of cube" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.95, 0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.95, -0.95);
  test(*a, *b, true);
  
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -0.95, 0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, -0.95, -0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.0, 0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.0, -0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.0, 0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.0, -0.95);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, 0.95, 0.0);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.95, -0.95, 0.0);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, 0.95, 0.0);
  test(*a, *b, true);

  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.95, -0.95, 0.0);
  test(*a, *b, true);

  cout << "Exact center" << endl;
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(0.0, 0.0, 0.0);
  test(*a, *b, true);


/**/
  
  /*
  a = new cube(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  b = new point(-0.99, -0.99, -0.50);
  test(*a, *b, true);
*/
}
