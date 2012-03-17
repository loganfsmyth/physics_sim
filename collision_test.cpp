
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

class line: public collidable {
  vec3 start, end;
  public:
  line(vec3 s, vec3 e) : start(s), end(e) { }
  vec3 collision_point(vec3 d) const {
    if ((end-start).dot(d) > 0) return end;
    else return start;
  }
};

class triangle: public collidable {
  vec3 a, b, c;
  public:
  triangle(vec3 na, vec3 nb, vec3 nc): a(na), b(nb), c(nc) { }
  vec3 collision_point(vec3 dir) const {
    vec3 centroid = (a+b+c) * (1.0/3);
    double tmp, angle = (a-centroid).dot(dir);
    vec3 closest = a;
    if ((tmp = (b-centroid).dot(dir)) > angle) {
      angle = tmp;
      closest = b;
    }
    if ((tmp = (c-centroid).dot(dir)) > angle) {
      angle = tmp;
      closest = c;
    }
    return closest;
  }
};

class tetrahedron: public collidable {
  vec3 a, b, c, d;
  public:
  tetrahedron(vec3 na, vec3 nb, vec3 nc, vec3 nd): a(na), b(nb), c(nc), d(nd) { }
  vec3 collision_point(vec3 dir) const {
    vec3 centroid = (a+b+c+d) * (1.0/4);
    double tmp, angle = (a-centroid).dot(dir);
    vec3 closest = a;
    if ((tmp = (b-centroid).dot(dir)) > angle) {
      angle = tmp;
      closest = b;
    }
    if ((tmp = (c-centroid).dot(dir)) > angle) {
      angle = tmp;
      closest = c;
    }
    if ((tmp = (d-centroid).dot(dir)) > angle) {
      angle = tmp;
      closest = d;
    }
    return closest;
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
//  cerr << endl;
}

void test_point_point() {

  collidable *a, *b;

  /**
   * Test point v point in 3d space.
   */
  cout << "point v point" << endl;
  a = new point(1, 0, 0);
  b = new point(1, 0, 0);
  test(*a, *b, true);

  b = new point(0,0,0);
  test(*a, *b, false);
  
  b = new point(2,0,0);
  test(*a, *b, false);

  b = new point(1,1,0);
  test(*a, *b, false);
  
  b = new point(1,-1,0);
  test(*a, *b, false);
  
  b = new point(1,0,1);
  test(*a, *b, false);
  
  b = new point(1,0,-1);
  test(*a, *b, false);
}

void test_point_line() {

  collidable *a, *b;

  /**
   * Test point v line in 3d space
   */
  cout << "point v line intersect" << endl;
  a = new point(0,0,0);
  b = new line(vec3(1, 0, 0), vec3(-1, 0, 0));
  test(*a, *b, true);

  b = new line(vec3(0, 1, 0), vec3(0, -1, 0));
  test(*a, *b, true);

  b = new line(vec3(0, 0, 1), vec3(0, 0, -1));
  test(*a, *b, true);

  b = new line(vec3(1, 1, 1), vec3(-1, -1, -1));
  test(*a, *b, true);
  
  b = new line(vec3(-1, 1, 1), vec3(1, -1, -1));
  test(*a, *b, true);
  
  b = new line(vec3(1, -1, 1), vec3(-1, 1, -1));
  test(*a, *b, true);
  
  b = new line(vec3(1, 1, -1), vec3(-1, -1, 1));
  test(*a, *b, true);
  
  // same tests with point slightly moved
  cout << "point v line no intersect" << endl;
  a = new point(0.01, 0.02, 0);
  b = new line(vec3(1, 0, 0), vec3(-1, 0, 0));
  test(*a, *b, false);

  b = new line(vec3(0, 1, 0), vec3(0, -1, 0));
  test(*a, *b, false);

  b = new line(vec3(0, 0, 1), vec3(0, 0, -1));
  test(*a, *b, false);

  b = new line(vec3(1, 1, 1), vec3(-1, -1, -1));
  test(*a, *b, false);
  
  b = new line(vec3(-1, 1, 1), vec3(1, -1, -1));
  test(*a, *b, false);
  
  b = new line(vec3(1, -1, 1), vec3(-1, 1, -1));
  test(*a, *b, false);
  
  b = new line(vec3(1, 1, -1), vec3(-1, -1, 1));
  test(*a, *b, false);

  // More specific point placements at end of line
  cout << "points at very ends of line" << endl;
  b = new line(vec3(1, 0, 0), vec3(-1, 0, 0));
  
  a = new point(1.01,0,0);
  test(*a, *b, false);

  a = new point(-1.01,0,0);
  test(*a, *b, false);
  
  a = new point(1.00,0,0);
  test(*a, *b, true);
  
  a = new point(-1.00,0,0);
  test(*a, *b, true);
  
  a = new point(0.99,0,0);
  test(*a, *b, true);
  
  a = new point(-0.99,0,0);
  test(*a, *b, true);
}

void test_point_triangle() {

  collidable *a, *b;

  /**
   * Test point v triangle in 3d space
   */
  cout << "point just beyond triangle front/back" << endl;
  b = new triangle(vec3(0, 1, 0), vec3(1, -1, 0), vec3(-1, -1, 0));
  // beyond front/back
  a = new point(0, 0, 0.01);
  test(*a, *b, false);
  
  a = new point(0, 0, -0.01);
  test(*a, *b, false);

  cout << "point on triangle plane" << endl;
  // Itersecting plane tests
  a = new point(0, 0, 0);
  b = new triangle(vec3(1, 0, 0), vec3(-1, 1, 0), vec3(-1, -1, 0));
  test(*a, *b, true);

  
  b = new triangle(vec3(0, 1, 0), vec3(0, -1, 1), vec3(0, -1, -1));
  test(*a, *b, true);

  b = new triangle(vec3(0, 0, 1), vec3(1, 0, -1), vec3(-1, 0, -1));
  test(*a, *b, true);

  

  cout << "point just beyond triangle corners" << endl;
  // Just beyond corners
  
  b = new triangle(vec3(0, 1, 0), vec3(1, -1, 0), vec3(-1, -1, 0));

  a = new point(0, 1.01, 0);
  test(*a, *b, false);
  
  a = new point(1.01, -1.01, 0);
  test(*a, *b, false);
  
  a = new point(-1.01, -1.01, 0);
  test(*a, *b, false);
  
  cout << "point just inside triangle corners" << endl;
  a = new point(0, 0.99, 0);
  test(*a, *b, true);
  
  a = new point(0.99, -0.99, 0);
  test(*a, *b, true);
  
  a = new point(-0.99, -0.99, 0);
  test(*a, *b, true);
  
  cout << "point just beyond triangle edges" << endl;
  // just beyond edges
  a = new point(0, -1.01, 0);
  test(*a, *b, false);
  
  a = new point(0.51, 0, 0);
  test(*a, *b, false);
  
  a = new point(-0.51, 0, 0);
  test(*a, *b, false);
  
  cout << "point just inside triangle edges" << endl;
  a = new point(0, -0.99, 0);
  test(*a, *b, true);
  
  a = new point(0.49, 0, 0);
  test(*a, *b, true);
  
  a = new point(-0.49, 0, 0);
  test(*a, *b, true);

  /**
   * Test point v tetrahedron in 3d space
   */
  cout << "point v tetrahedron" << endl;

  b = new tetrahedron(vec3(0, 1, 0), vec3(1, -1, 0), vec3(-1, -1, 1), vec3(-1, -1, -1));

  a = new point(0,0,0);
  test(*a, *b, true);

  cout << "point inside faces" << endl;
  a = new point(0.0, -0.30, 0.3);
  test(*a, *b, true);

  a = new point(0.0, -0.33, -0.33);
  test(*a, *b, true);

  a = new point(-0.66, -0.33, 0);
  test(*a, *b, true);

  a = new point(-0.33, -0.99, 0);
  test(*a, *b, true);

  cout << "point outside faces" << endl;
  a = new point(0.0, -0.35, 0.35);
  test(*a, *b, false);
  
  a = new point(0.0, -0.35, -0.35);
  test(*a, *b, false);

  a = new point(-0.68, -0.35, 0);
  test(*a, *b, false);

  a = new point(-0.35, -1.03, 0);
  test(*a, *b, false);

  cout << "point inside edge" << endl;

  // ab edge
  a = new point(0.49, 0, 0);
  test(*a, *b, true);

  // ac edge
  a = new point(-0.49, 0, 0.49);
  test(*a, *b, true);

  // ad edge
  a = new point(-0.49, 0, -0.49);
  test(*a, *b, true);

  // bc edge
  a = new point(0, -0.99, 0.49);
  test(*a, *b, true);

  // bd edge
  a = new point(0, -0.99, -0.49);
  test(*a, *b, true);

  // cd edge
  a = new point(-0.99, -0.99, 0);
  test(*a, *b, true);

  cout << "point outside edge" << endl;

  // ab edge
  a = new point(0.51, 0, 0);
  test(*a, *b, false);

  // ac edge
  a = new point(-0.51, 0, 0.51);
  test(*a, *b, false);

  // ad edge
  a = new point(-0.51, 0, -0.51);
  test(*a, *b, false);

  // bc edge
  a = new point(0, -1.01, 0.51);
  test(*a, *b, false);

  // bd edge
  a = new point(0, -1.01, -0.51);
  test(*a, *b, false);

  // cd edge
  a = new point(-1.01, -1.01, 0);
  test(*a, *b, false);

  cout << "point outside corner" << endl;
  a = new point(0, 1.01, 0);
  test(*a, *b, false);

  a = new point(1.01, -1.01, 0);
  test(*a, *b, false);

  a = new point(-1.01, -1.01, 1.01);
  test(*a, *b, false);

  a = new point(-1.01, -1.01, -1.01);
  test(*a, *b, false);

  cout << "point inside corner" << endl;
  a = new point(0, 0.99, 0);
  test(*a, *b, true);

  a = new point(0.99, -0.99, 0);
  test(*a, *b, true);

  a = new point(-0.99, -0.99, 0.99);
  test(*a, *b, true);

  a = new point(-0.99, -0.99, -0.99);
  test(*a, *b, true);


}

void test_point_tetrahedron() {

  collidable *a, *b;

  /**
   * Test point v tetrahedron in 3d space
   */
  cout << "point v tetrahedron" << endl;

  b = new tetrahedron(vec3(0, 1, 0), vec3(1, -1, 0), vec3(-1, -1, 1), vec3(-1, -1, -1));

  a = new point(0,0,0);
  test(*a, *b, true);

  cout << "point inside faces" << endl;
  a = new point(0.0, -0.30, 0.3);
  test(*a, *b, true);

  a = new point(0.0, -0.33, -0.33);
  test(*a, *b, true);

  a = new point(-0.66, -0.33, 0);
  test(*a, *b, true);

  a = new point(-0.33, -0.99, 0);
  test(*a, *b, true);

  cout << "point outside faces" << endl;
  a = new point(0.0, -0.35, 0.35);
  test(*a, *b, false);
  
  a = new point(0.0, -0.35, -0.35);
  test(*a, *b, false);

  a = new point(-0.68, -0.35, 0);
  test(*a, *b, false);

  a = new point(-0.35, -1.03, 0);
  test(*a, *b, false);

  cout << "point inside edge" << endl;

  // ab edge
  a = new point(0.49, 0, 0);
  test(*a, *b, true);

  // ac edge
  a = new point(-0.49, 0, 0.49);
  test(*a, *b, true);

  // ad edge
  a = new point(-0.49, 0, -0.49);
  test(*a, *b, true);

  // bc edge
  a = new point(0, -0.99, 0.49);
  test(*a, *b, true);

  // bd edge
  a = new point(0, -0.99, -0.49);
  test(*a, *b, true);

  // cd edge
  a = new point(-0.99, -0.99, 0);
  test(*a, *b, true);

  cout << "point outside edge" << endl;

  // ab edge
  a = new point(0.51, 0, 0);
  test(*a, *b, false);

  // ac edge
  a = new point(-0.51, 0, 0.51);
  test(*a, *b, false);

  // ad edge
  a = new point(-0.51, 0, -0.51);
  test(*a, *b, false);

  // bc edge
  a = new point(0, -1.01, 0.51);
  test(*a, *b, false);

  // bd edge
  a = new point(0, -1.01, -0.51);
  test(*a, *b, false);

  // cd edge
  a = new point(-1.01, -1.01, 0);
  test(*a, *b, false);

  cout << "point outside corner" << endl;
  a = new point(0, 1.01, 0);
  test(*a, *b, false);

  a = new point(1.01, -1.01, 0);
  test(*a, *b, false);

  a = new point(-1.01, -1.01, 1.01);
  test(*a, *b, false);

  a = new point(-1.01, -1.01, -1.01);
  test(*a, *b, false);

  cout << "point inside corner" << endl;
  a = new point(0, 0.99, 0);
  test(*a, *b, true);

  a = new point(0.99, -0.99, 0);
  test(*a, *b, true);

  a = new point(-0.99, -0.99, 0.99);
  test(*a, *b, true);

  a = new point(-0.99, -0.99, -0.99);
  test(*a, *b, true);


}

void test_point_cube() {

  collidable *a, *b;

  /**
   * Test point v cube in 3d space
   */
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

}

void test_sphere_sphere() {

  collidable *a, *b;

  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.1);
  test(*a, *b, true);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 0.9);
  test(*a, *b, false);
  
  a = new sphere(-1, 0, 0, 1);
  b = new sphere(1, 0, 0, 1.0);
  test(*a, *b, true);

}


int main(int argc, char** argv) {

  test_point_point();
  test_point_line();
  test_point_triangle();
  test_point_tetrahedron();
  test_point_cube();
  test_sphere_sphere();


}
