
#include <list>
#include <set>
#include <limits>
#include <iostream>
#include "collision.h"
#include <cmath>
#include <cassert>

using std::cerr;
using std::cerr;
using std::endl;
using namespace std;

#define COLLISION_DEBUG 1

#ifndef COLLISION_DEBUG
#define COLLISION_DEBUG 0
#endif

simplex_pt::simplex_pt(vec3 v, vec3 na, vec3 nb): val(v), a(na), b(nb) { }
simplex_pt::simplex_pt() { }

simplex_pt collision_vec(vec3 dir, const collidable &a, const collidable &b) {
  vec3 one = a.collision_point(dir);
  vec3 two = b.collision_point(dir*-1);
  vec3 s = one-two;
  
#if COLLISION_DEBUG
//  cerr << "one: " << one << " two: " << two << " dir: " << dir << " sum: " << s << endl;
#endif
  simplex_pt val(s,one,two);
  return val;
}

bool process_simplex(std::vector<simplex_pt> &pts, vec3 &dir) {
  switch (pts.size()) {
    case 0:
      break;
    case 1: {
      vec3 &a = pts[0].val;
      break;
    }
    case 2: {
      vec3 &a = pts[1].val;
      vec3 &b = pts[0].val;
      vec3 ab = b - a;
      vec3 a0 = a * -1;

      double dist = ab.dot(a0);

      vec3 v = (ab*a0);
#if COLLISION_DEBUG
      cerr << "ab:" << ab << " = " << "a0:" <<a0 << " = v:" << v << " =1" << endl;
#endif

      if (dist == 0) { // avoid (0,0,0) cross product for point v point case
        dir *= -1;
      }
      else if (dist > 0) {
        dir = (v * ab);
      }
      else {
        pts[0] = pts[1];
        pts.pop_back();
        dir = a0;
      }
      break;
    }
    case 3: {
      vec3 &a = pts[2].val;
      vec3 &b = pts[1].val;
      vec3 &c = pts[0].val;
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 a0 = a * -1;
      vec3 abc = (ab*ac);
      

#if COLLISION_DEBUG
      cerr << "triangle: a: " << a << " b: " << b << " c: " << c << endl;
//      cerr << "\t\tabc: " << abc << " ab: " << ab << " ac: " << ac << " a0: " << a0 << endl;
      cerr << "triangle";
#endif
      if ((abc*ac).dot(a0) > 0) { // ac or ab edge or a corner
#if COLLISION_DEBUG
        cerr << ".1";
#endif
        if (ac.dot(a0) > 0) { // ac edge
#if COLLISION_DEBUG
          cerr << ".1";
#endif
          // pts => c,a
          pts[1] = pts[2];
          pts.pop_back();
          dir = ac * a0 * ac;
        }
        else {
#if COLLISION_DEBUG
          cerr << ".2";
#endif
          if (ab.dot(a0) > 0) { // ab edge
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            // pts => b,a
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => a
            pts[0] = pts[2];
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
      }
      else {
#if COLLISION_DEBUG
        cerr << ".2";
#endif
        if ((ab*abc).dot(a0) > 0) { // ab edge or a corner
#if COLLISION_DEBUG
          cerr << ".1";
#endif
          if (ab.dot(a0) > 0) { // ab edge
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            // pts => b,a
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => a
            pts[0] = pts[2];
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
        else { // inside triangle, above or below
#if COLLISION_DEBUG
          cerr << "-2";
#endif
          double v = abc.dot(a0);
          if (v == 0) {
            dir = a0;
#if COLLISION_DEBUG
            cerr << endl;
#endif
            return true;
          }
          else if (v > 0) { // above  points 
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            // pts => c,b,a
            dir = abc;
          }
          else { // below
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => b,c,a
            simplex_pt tmp = pts[0];
            pts[0] = pts[1];
            pts[1] = tmp;
            dir = abc * -1;
          }
        }
      }
#if COLLISION_DEBUG
      cerr << endl;
#endif
      break;
    }
    case 4: {
      vec3 &a = pts[3].val;
      vec3 &b = pts[2].val;
      vec3 &c = pts[1].val;
      vec3 &d = pts[0].val;
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 ad = d - a;
      vec3 a0 = a * -1;
      vec3 abc = (ab*ac);
      vec3 abd = (ab*ad); // This vector points INWARD
      vec3 acd = (ac*ad);

#if COLLISION_DEBUG
      cerr << "ab: " << ab << " ac: " << ac << " ad: " << ad << " a0: " << a0 << "=2" << endl;
      cerr << "abc: " << abc << " abd: " << abd << " acd: " << acd << endl;

      cerr << "tetra";
#endif

      if (abc.dot(a0) > 0) { // bc edges/corners excluded by prior info
#if COLLISION_DEBUG
        cerr << ".1";
#endif
        if ((ab * abc).dot(a0) > 0) { // abd face, ab edge or a corner
#if COLLISION_DEBUG
          cerr << ".1";
#endif
          if (ab.dot(a0) > 0) { // ab edge or abd face
#if COLLISION_DEBUG
            cerr << "-1";
#endif
            if ((ab*abd).dot(a0) > 0) { // ab edge
#if COLLISION_DEBUG
              cerr << ".1";
#endif
              // pts => b,a
              pts[0] = pts[2];
              pts[1] = pts[3];
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // abd face
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => b,d,a, wound opposite
              simplex_pt tmp = pts[0];
              pts[0] = pts[1];
              pts[1] = tmp;
              pts[2] = pts[3];
              pts.pop_back();
              dir = abd * -1;
            }
          }
          else { // TODO: Check all these conditions
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            if (ad.dot(a0) < 0) {
#if COLLISION_DEBUG
              cerr << ".1";
#endif
              if (ac.dot(a0) < 0) { // a corner
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // acd face
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
            else { // abd face
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => b,d,a, wound opposite
              simplex_pt tmp = pts[0];
              pts[0] = pts[1];
              pts[1] = tmp;
              pts[2] = pts[3];
              pts.pop_back();
              dir = abd * -1;
            }
          }
        }
        else {
#if COLLISION_DEBUG
          cerr << ".2";
#endif
          if ((abc*ac).dot(a0) > 0) { // ac edge or a corner
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            if (ac.dot(a0) > 0) { // ac edge
#if COLLISION_DEBUG
              cerr << "-1";
#endif
              if ((ac*acd).dot(a0) > 0) { // ac edge
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                // pts => c,a
                pts[0] = pts[1];
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // acd face
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
            else { // TODO Can this be abd face too?
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              if (ad.dot(a0) < 0) { // a corner
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // ad edge
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => d,a
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ad*a0*ad;
              }
            }
          }
          else { // abc face
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => c,b,a
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts[2] = pts[3];
            pts.pop_back();
            dir = abc;
          }
        }
      }
      else { // abc face out
#if COLLISION_DEBUG
        cerr << ".2";
#endif
        if (abd.dot(a0) < 0) { // abd plane, bd edges/corders excluded by prior info
#if COLLISION_DEBUG
          cerr << ".1";
#endif
          if ((ab*abd).dot(a0) > 0) { // ab edge or a corner
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            if (ab.dot(a0) > 0) { // ab edge
#if COLLISION_DEBUG
              cerr << ".1";
#endif
              // pts => b,a
              pts[0] = pts[2];
              pts[1] = pts[3];
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // a corner
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => a
              pts[0] = pts[3];
              pts.pop_back();
              pts.pop_back();
              pts.pop_back();
              dir = a0;
            }
          }
          else {
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            if ((abd*ad).dot(a0) > 0) { // ad edge or a corner
#if COLLISION_DEBUG
              cerr << ".1";
#endif
              if (ad.dot(a0) > 0) {
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                if ((acd*ad).dot(a0) > 0) { // ad edge
#if COLLISION_DEBUG
                  cerr << ".1";
#endif
                  // pts => d,a
                  pts[1] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // acd face
#if COLLISION_DEBUG
                  cerr << ".2";
#endif
                  // pts => d,c,a
                  pts[2] = pts[3];
                  pts.pop_back();
                  dir = acd;
                }
              }
              else { // a corner
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else { // abd face
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => b,d,a, wound opposite
              simplex_pt tmp = pts[0];
              pts[0] = pts[2];
              pts[1] = tmp;
              pts[2] = pts[3];
              pts.pop_back();
              dir = abd * -1;
            }
          }
        }
        else { // abd face out
#if COLLISION_DEBUG
          cerr << ".2";
#endif
          if (acd.dot(a0) > 0) { // acd plane, cd edges/corners excluded by prior info
#if COLLISION_DEBUG
            cerr << ".1";
#endif
            if ((ac*acd).dot(a0) > 0) { // ac edge or a corner
#if COLLISION_DEBUG
              cerr << ".1";
#endif
              if (ac.dot(a0) > 0) { // ac edge
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                // pts => c,a
                pts[0] = pts[1];
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // a corner
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else {
#if COLLISION_DEBUG
              cerr << "-2";
#endif
              if ((acd*ad).dot(a0) > 0) { // ad edge or a corner
#if COLLISION_DEBUG
                cerr << ".1";
#endif
                if (ad.dot(a0) > 0) { // ad edge
#if COLLISION_DEBUG
                  cerr << ".1";
#endif
                  // pts => d,a
                  pts[1] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // a corner
#if COLLISION_DEBUG
                  cerr << ".2";
#endif
                  // pts => a
                  pts[0] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  pts.pop_back();
                  dir = a0;
                }
              }
              else { // acd face
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
          }
          else {
#if COLLISION_DEBUG
            cerr << "-2";
            cerr << endl;
#endif
            return true;
          }
        }
      }
#if COLLISION_DEBUG
      cerr << endl;
#endif
      break;
    }
  }

  return false;
}


bool collide(const collidable &a, const collidable &b, std::vector<simplex_pt> &pts, vec3 &dir) {
  simplex_pt n,
             pt = collision_vec(vec3(1.0f, 0.0f, 0.0f), a, b);

  pts.reserve(4);
  pts.push_back(pt);
  dir = pt.val * -1;
  while (true) {
    n = collision_vec(dir, a, b);
    if (n.val.dot(dir) < 0) return false;
    pts.push_back(n);
    if (process_simplex(pts, dir)) return true;
  }
  return false;
}

bool collide(const collidable &a, const collidable &b) {
  std::vector<simplex_pt> pts;
  vec3 dir;
  return collide(a, b, pts, dir);
}

void rotateVec(vec3 &v, double angle, const vec3 &ax) {
  double c = cos(angle);
  double s = sin(angle);

  double x = v.x*c;
  double y = v.y*c;
  double z = v.z*c;

  x += s * (-1*ax.z*v.y + ax.y*v.z);
  y += s * (ax.z*v.x - ax.x*v.z);
  z += s * (-1*ax.y*v.x + ax.x*v.y);

  x += (1-c) * ( ax.x*ax.x*v.x + ax.x*ax.y*v.y + ax.x*ax.z*v.z );
  y += (1-c) * ( ax.x*ax.y*v.x + ax.y*ax.y*v.y + ax.y*ax.z*v.z );
  z += (1-c) * ( ax.x*ax.z*v.x + ax.y*ax.z*v.y + ax.z*ax.z*v.z );

  v.x = x;
  v.y = y;
  v.z = z;
}


list<vec3> collision_points(const collidable &a, const vec3 &n, vec3 perp, const vec3 &pt, int samples) {
  cout << "Testing Normal " << n << " with " << samples << " samples" << endl;
  double angle = 2*3.1415926535 / samples;
  list<vec3> pts;
  for (int i = 0; i < samples; i++) {
    vec3 to_vert = a.collision_point(n + perp) - pt;
    vec3 on_plane = to_vert - n * n.dot(to_vert);
    bool found = false;
    for (list<vec3>::iterator it = pts.begin(); it != pts.end(); it++) {
      if (on_plane == *it) found = true;
    }

    if (!found) {
      pts.push_back(on_plane);
    }
    rotateVec(perp, angle, n);
  }

  return pts;
}




void closest_simplex(const collidable &a, const collidable &b, std::vector<simplex_pt> &pts) {
  simplex_pt n = collision_vec(vec3(1, 0, 0), a, b);
  cerr << "val: " << n.val << " a:" << n.a << " b:" << n.b << endl;
  pts.reserve(3);
  pts.push_back(n);
  vec3 dir = n.val * -1;
  while (true) {
    dir.norm();
    n = collision_vec(dir, a, b);
    cerr << "val: " << n.val << " a:" << n.a << " b:" << n.b << endl;
    if (n.val.dot(dir) - pts.back().val.dot(dir) < 0.1) {
      break;
    }
    pts.push_back(n);
    process_simplex(pts, dir);
  }
}


bool simplex_pt::operator==(const simplex_pt &p) {
  return a == p.a && b == p.b;
}


epa_tri::epa_tri(simplex_pt &a, simplex_pt &b, simplex_pt &c) : a(a), b(b), c(c) {
  dist = -1;
  ab = b.val-a.val;
  ac = c.val-a.val;
  bc = c.val-b.val;
  norm = ab*ac;
  norm.norm();
}

list<epa_tri>::iterator epa_min_dist(std::list<epa_tri> &tris) {
  double min = numeric_limits<double>::infinity();
  list<epa_tri>::iterator closest;

  for (list<epa_tri>::iterator it = tris.begin(); it != tris.end(); it++) {
    if (it->dist == -1) {
      it->dist = it->a.val.dot(it->norm);
      if (it->dist < 0) {
        cerr << "LT!!!!" << endl;
      }
    }
    if (it->dist < min) {
      closest = it;
      min = it->dist;
      cerr << "MIN" << endl;
    }
    cerr << ">v " << it->a.val << it->b.val << it->c.val << it->norm << " dist" << it->dist << endl;
  }
  return closest;
}

struct hull_edge {
  int a, b;
  int refs;
  hull_edge(int a, int b) : a(a), b(b) {
    refs = 0;
  }
};
struct hull_face {
  int e1, e2, e3;
  bool removed;
  hull_face(int e1, int e2, int e3) : e1(e1), e2(e2), e3(e3) {
    removed = false;
  }
};


class chull {
  typedef simplex_pt spt;
  typedef vector<simplex_pt> plist;
  typedef vector<hull_edge> elist;
  typedef vector<hull_face> flist;

  plist pts;
  elist edges;
  flist faces;

  public:
  chull(const spt &a, const spt &b, const spt &c, const spt &d) {
    add_pt(a);
    add_pt(b);
    add_pt(c);
    add_pt(d);
  }
  void add_pt(const spt &p) {
    cerr << "Adding point " << p.val << endl;

    for (plist::iterator it = pts.begin(); it != pts.end(); it++) {
      if (it->val == p.val) return;
    }

    pts.push_back(p);
    int pos = pts.size()-1;

    if (pos == 0) {
      return;
    }
    else if (edges.size() == 0) {
      addEdge(0, pos);
      return;
    }
    else if (edges.size() == 1) {
      int e1 = addEdge(0,pos);
      int e2 = addEdge(1,pos);
      addFace(0, e1, e2);
      return;
    }
    else if (faces.size() == 1) {
      int e1 = addEdge(0,pos);
      int e2 = addEdge(1,pos);
      int e3 = addEdge(2,pos);
      addFace(0, e1, e2);
      addFace(1, e1, e3);
      addFace(2, e2, e3);
      return;
    }

    list<int> possible_edges;
    for (flist::iterator it = faces.begin(); it != faces.end(); it++) {
      if (!it->removed && (p.val - pts[edges[it->e1].a].val).dot(fNorm(*it)) > 0) { // find faces pointing toward new pt
        // Remove face toward new point, and add edges as possible edges for new triangles
        fRemove(*it);
        possible_edges.push_back(it->e1);
        possible_edges.push_back(it->e2);
        possible_edges.push_back(it->e3);
      }
    }

    // Check edges of removed faces for ones still references. They are the ones along edges.
    for (list<int>::iterator it = possible_edges.begin(); it != possible_edges.end(); it++) {
      hull_edge &e = edges[*it];
      cerr << "C";
      pEdge(e);
      if (e.refs > 0) {
        int e1 = getEdge(e.a, pos);
        int e2 = getEdge(e.b, pos);
        addFace(*it, e1, e2);
      }
    }
  }
  void addFace(int e1, int e2, int e3) {
    faces.push_back(hull_face(e1, e2, e3));
    edges[e1].refs += 1;
    edges[e2].refs += 1;
    edges[e3].refs += 1;

    cerr << "Add Face ";
    pFace(faces.size()-1);
  }
  int addEdge(int a, int b) {
    edges.push_back(hull_edge(a,b));
    return edges.size()-1;
  }
  int getEdge(int a, int b) {
    int i = 0;
    for (elist::iterator it = edges.begin(); it != edges.end(); it++, i++) {
      if ((it->a == a && it->b == b) || (it->b == a && it->a == b)) {
        return i;
      }
    }
    addEdge(a,b);
    return edges.size()-1;
  }
  void fRemove(hull_face &f) {
    cerr << "Remove face ";
    pFace(f);

    f.removed = true;
    edges[f.e1].refs -= 1;
    edges[f.e2].refs -= 1;
    edges[f.e3].refs -= 1;
  }
  vec3 fNorm(int f) {
    return fNorm(faces[f]);
  }
  vec3 fNorm(hull_face &f) {
//    cerr << "Norm: " << f.e1 << " " << f.e2 << " " << f.e3 << endl;
//    cerr << "E" << pts[edges[f.e1].a].val << pts[edges[f.e1].b].val << endl;
//    cerr << "E" << pts[edges[f.e2].a].val << pts[edges[f.e2].b].val << endl;
//    cerr << "E" << pts[edges[f.e3].a].val << pts[edges[f.e3].b].val << endl;


    hull_edge &ab = edges[f.e1];
    hull_edge &ac = edges[f.e2];
    vec3 norm = (pts[ab.b].val - pts[ab.a].val) * (pts[ac.b].val - pts[ac.a].val);
    if (pts[ab.a].val.dot(norm) < 0) {
      norm *= -1;
    }
    norm.norm();
//    cerr << norm << endl;
    return norm;
  }
  pair<double,int> closestFace() {
    double min = numeric_limits<double>::infinity();
    int face = -1;
    int i = 0;
    for (flist::iterator it = faces.begin(); it != faces.end(); it++, i++) {
      if (it->removed) continue;
      double dist = fNorm(*it).dot(pts[edges[it->e1].a].val);

      if (dist < min) {
        min = dist;
        face = i;
      }
    }
    cerr << "Closest ";
    pFace(face);
    return pair<double,int>(min,face);
  }

  void pFace(int f) {
    pFace(faces[f]);
  }
  void pFace(hull_face &f) {
    cerr << "Face: N:" << fNorm(f) << endl;
    pEdge(f.e1);
    pEdge(f.e2);
    pEdge(f.e3);
  }
  void pEdge(int e) {
    pEdge(edges[e]);
  }
  void pEdge(hull_edge &e) {
    cerr << " Edge:" << pts[e.a].val << " " << pts[e.b].val << " Refs:" << e.refs << endl;
  }

  epa_tri getTri(int fid) {
    hull_face &f = faces[fid];
    hull_edge &e1 = edges[f.e1],
              &e2 = edges[f.e2],
              &e3 = edges[f.e3];

    set<int> s;
    s.insert(e1.a);
    s.insert(e1.b);
    s.insert(e2.a);
    s.insert(e2.b);
    s.insert(e3.a);
    s.insert(e3.b);
    assert(s.size() == 3);
    set<int>::iterator it = s.begin();
    spt *a = &pts[*it++],
        *b = &pts[*it++],
        *c = &pts[*it++];

    // Fix winding so normal points away from origin
    if (((b->val - a->val)*(c->val - a->val)).dot(b->val) < 0) {
      swap(b,c);
    }

    return epa_tri(*a,*b,*c);
  }
};


epa_tri epa(const collidable &one, const collidable &two, vector<simplex_pt> &pts) {

  simplex_pt &a = pts[3],
             &b = pts[2],
             &c = pts[1],
             &d = pts[0];

  cerr << "a:" << a.a << b.a << c.a << d.a << endl;
  cerr << "b:" << a.b << b.b << c.b << d.b << endl;
  cerr << "val" << a.val << b.val << c.val << d.val << endl;

  chull h(a, b, c, d);

  int fid;
  while (true) {
    pair<double,int> face = h.closestFace();
    fid = face.second;
    vec3 norm = h.fNorm(fid);

    simplex_pt p = collision_vec(norm, one, two);
    double d = p.val.dot(norm);

    cerr << "P" << p.val << norm << " d:" << d << " f:" << face.first << endl;

    if (d - face.first < 0.001) {
      break;
    }
    else {
      h.add_pt(p);
    }
  }

  return h.getTri(fid);
}

typedef pair<vec3,vec3> edge;

vec3 find_intersection(edge &one, edge &two) {
  vec3 v1 = one.second - one.first,
       v2 = two.second - two.first;

  double l = ((two.first - one.first) * v2).len() / (v1*v2).len();

  return one.first + v1 * l;
}

void calculate_overlap(list<edge> &a_edges, list<edge> &b_edges, const vec3 &n) {

  for (list<edge>::iterator it = a_edges.begin(); it != a_edges.end(); it++) {
    vec3 left = it->first,
          right = it->second,
          norm = (right-left)*n;

    bool rem = false;
    for (list<edge>::iterator it2 = b_edges.begin(); it2 != b_edges.end(); it2++) {
      vec3 left2 = it2->first,
            right2 = it2->second,
            b_norm = (right2 - left2) * n;

      bool l = norm.dot(left - left2) > 0;
      bool r = norm.dot(left - right2) > 0;
      if (!l && !r) {
        it2 = --b_edges.erase(it2);
        rem = true;
        continue;
      }

      bool overlap = (l ^ r);
      l = (left2 - left).dot(b_norm) > 0;
      r = (left2 - right).dot(b_norm) > 0;

      if (overlap && l^r) {

        vec3 in = find_intersection(*it, *it2);
      
        if ((left - in).dot(b_norm) > 0) {
          it->first = in;
        }
        else {
          it->second = in;
        }
        
        if ((left2 - in).dot(norm) > 0) {
          it2->first = in;
        }
        else {
          it2->second = in;
        }
      }
    }

    if (rem) {
      it = --a_edges.erase(it);
    }
  }
}



bool contact_points(collidable &a, collidable &b, list<vec3> &a_pts, list<vec3> &b_pts, vec3 &sep) {
  

  vector<simplex_pt> sim;
  vec3 sep_axis;
  if (!collide(a,b,sim, sep_axis)) return false;
  sep = sep_axis;

  epa_tri t = epa(a, b, sim);
  vec3 n = t.norm;

  vec3 perp = n*vec3(1.12345, 0.6543, 0.987564);
  perp *= 0.1 / perp.len();

  sep = n;

  list<vec3> cpts = collision_points(a, n, perp, t.a.a, 20);
  vec3 inv = n*-1;
  list<vec3> cpts2 = collision_points(b, inv, perp, t.a.b, 10);

  cerr << distance(cpts.begin(), cpts.end()) << " -- " << distance(cpts2.begin(), cpts2.end()) << endl;

  list<edge> a_edges;
  for (list<vec3>::iterator it = cpts.begin(); it != cpts.end(); it++) {
    list<vec3>::iterator tmp = it;
    tmp++;
    if (tmp == cpts.end()) tmp = cpts.begin();
    a_edges.push_back(edge(t.a.a + *it, t.a.a + *tmp));
  }

  list<edge> b_edges;
  for (list<vec3>::iterator it = cpts2.begin(); it != cpts2.end(); it++) {
    list<vec3>::iterator tmp = it;
    tmp++;
    if (tmp == cpts2.end()) tmp = cpts2.begin();
    b_edges.push_back(edge(t.a.b + *it, t.a.b + *tmp));
  }


  // Swap B coords to find winding direction to be same as 'A'.
  for (list<edge>::iterator it = b_edges.begin(); it != b_edges.end(); it++) {
    swap(it->first, it->second);
  }
  cout << "DONE" << endl;

//  calculate_overlap(a_edges, b_edges, n);

  cout << "DONE1" << endl;
  for (list<edge>::iterator it = a_edges.begin(); it != a_edges.end(); it++) {
    a_pts.push_back(it->first);
    a_pts.push_back(it->second);

    a.sim_edges.push_back(*it);
  }
  cout << "DONE2" << endl;
  
  for (list<edge>::iterator it = b_edges.begin(); it != b_edges.end(); it++) {
    b_pts.push_back(it->first);
    b_pts.push_back(it->second);

    b.sim_edges.push_back(*it);
  }
  cout << "DONE3" << endl;
  return true;
}


