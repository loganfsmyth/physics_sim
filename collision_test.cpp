
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
    }
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
        double n = it->dot(dir);
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

void test(const collidable &a, const collidable &b, bool ex) {
  bool hit = collide(a, b);
  cout << (hit == ex ? "PASS: " : "FAIL: ") << (hit ? "HIT" : "NO HIT") << endl;
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

void two_simplex(bool ex, string s, vec3 origin, vec3 exdir) {
  vec3 a = vec3( 0, 1, 0) - origin,
       b = vec3( 1,-1, 0) - origin,
       c = vec3(-1,-1, 0) - origin,
       dir;

  std::vector<simplex_pt> sim;
  sim.reserve(4);
  sim.push_back(simplex_pt(c, vec3(), vec3()));
  sim.push_back(simplex_pt(b, vec3(), vec3()));
  sim.push_back(simplex_pt(a, vec3(), vec3()));

  bool res = process_simplex(sim, dir);
  bool fail = (res != ex);
  int i = 0;
  for (string::iterator it = s.begin(); it != s.end(); it++, i++) {
    switch (*it) {
      case 'a':
        if (a != sim[i].val) fail = true;
        break;
      case 'b':
        if (b != sim[i].val) fail = true;
        break;
      case 'c':
        if (c != sim[i].val) fail = true;
        break;
    }
  }

  if (s.size() != sim.size()) fail = true;
  if (dir.dot(exdir)/(dir.len()*exdir.len()) < 0.99) fail = true;

  cout << (fail ? "FAIL" : "PASS") << endl;
}

void test_two_simplex() {

  two_simplex(true, "cba", vec3(0,0,0), vec3()); // on plane at origin

  two_simplex(false, "ba", vec3(1,0,0), vec3(2, 1, 0)); // off to left side
  two_simplex(false, "ca", vec3(-1,0,0), vec3(-2, 1, 0)); // off to right side
  // bottom side is not checked because it cannot be

  two_simplex(false, "a", vec3(0,1.5,0), vec3(0, 0.5, 0)); // off top corner
  // bottom two corners are not checked because they can't be true

  two_simplex(false, "bca", vec3(0,0,1), vec3(0, 0, 1)); // above triangle
  two_simplex(false, "cba", vec3(0,0,-1), vec3(0, 0, -1)); // below triangle
}

void three_simplex(bool ex, string s, vec3 origin, vec3 exdir) {
  vec3 a = vec3( 0, 0, 1) - origin,
       b = vec3( 0, 1,-1) - origin,
       c = vec3(-1,-1,-1) - origin,
       d = vec3( 1,-1,-1) - origin,
       dir;

  std::vector<simplex_pt> sim;
  sim.reserve(4);
  sim.push_back(simplex_pt(d, vec3(), vec3()));
  sim.push_back(simplex_pt(c, vec3(), vec3()));
  sim.push_back(simplex_pt(b, vec3(), vec3()));
  sim.push_back(simplex_pt(a, vec3(), vec3()));

  bool res = process_simplex(sim, dir);
  bool fail = (res != ex);
//  cerr << fail << endl;
  int i = 0;
  for (string::iterator it = s.begin(); it != s.end(); it++, i++) {
    switch (*it) {
      case 'a':
        if (a != sim[i].val) fail = true;
//        cerr << fail << endl;
        break;
      case 'b':
        if (b != sim[i].val) fail = true;
//        cerr << fail << endl;
        break;
      case 'c':
        if (c != sim[i].val) fail = true;
//        cerr << fail << endl;
        break;
      case 'd':
        if (d != sim[i].val) fail = true;
//        cerr << fail << endl;
        break;
    }
  }

//  cerr << dir << endl;

  if (s.size() != sim.size()) fail = true;
//  cerr << fail << endl;
  if (dir.dot(exdir)/(dir.len()*exdir.len()) < 0.99) fail = true;
//  cerr << fail << endl;

  cout << (fail ? "FAIL" : "PASS") << endl;
}

void test_three_simplex() {

  // Inside center
  three_simplex(true, "dcba", vec3(0,0,0), vec3(0,0,-1));

  // Faces
  // Beyond faces
  three_simplex(false, "cba", vec3(-0.34,0,-0.34), vec3(-4,2,1));
  three_simplex(false, "bda", vec3( 0.34,0,-0.34), vec3( 4,2,1));
  three_simplex(false, "dca", vec3( 0,-0.68,-0.34), vec3( 0,-4,2));
  // Inside faces
  three_simplex(true, "dcba", vec3(-0.33,0,-0.33), vec3());
  three_simplex(true, "dcba", vec3( 0.33,0,-0.33), vec3());
  three_simplex(true, "dcba", vec3( 0,-0.66,-0.33), vec3());
  // bottom face covered by 3-simplex checks

  // Edges
  // Beydon edges
  three_simplex(false, "ba", vec3(0, 1, 0), vec3(0, 2, 1));
  three_simplex(false, "ca", vec3(-1, -1, 0), vec3(-2, -2, 2));
  three_simplex(false, "da", vec3(1, -1, 0), vec3(2, -2, 2));
  
  // inside edges
  three_simplex(true, "dcba", vec3(0, 0.33, 0), vec3());
  three_simplex(true, "dcba", vec3(-0.33, -0.33, 0), vec3());
  three_simplex(true, "dcba", vec3( 0.33, -0.33, 0), vec3());
  // bc, bd, cd covered by 2 and 3-simplex checks


  // Above 'a'
  three_simplex(false, "a", vec3(0, 0, 2), vec3(0, 0, 1));

  // Inside 'a' corner
  three_simplex(true, "dcba", vec3(0, 0, 1), vec3());

}


int main(int argc, char** argv) {

  test_two_simplex();

  test_three_simplex();

/**/
  test_point_point();
  test_point_line();
  test_point_triangle();
  test_point_tetrahedron();
  test_point_cube();
  test_sphere_sphere();
/**/

  return 0;
}
