
#include <list>
#include <limits>
#include <iostream>
#include "collision.h"
#include <cmath>

using std::cout;
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
      cerr << "ab:" << ab << " = " << "a0:" <<a0 << " = v:" << v << endl;
#endif

      if (dist > 0) {
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
      cerr << "ab: " << ab << " ac: " << ac << " ad: " << ad << " a0: " << a0 << endl;
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


list<vec3> collision_points(const collidable &a, vec3 &n, vec3 perp, vec3 &pt, int samples) {
  double angle = 2*3.1415926535 / samples;
  list<vec3> pts;
  for (int i = 0; i < samples; i++) {
    vec3 to_vert = a.collision_point(n + perp) - pt;
    vec3 on_plane = to_vert - n * n.dot(to_vert);
    bool found = false;
    for (list<vec3>::iterator it = pts.begin(); it != pts.end(); it++) {
      if (to_vert == *it) found = true;
    }
    if (!found) {
      pts.push_back(to_vert);
    }
    rotateVec(perp, angle, n);
  }

  return pts;
}




void closest_simplex(const collidable &a, const collidable &b, std::vector<simplex_pt> &pts) {
  simplex_pt n = collision_vec(vec3(1, 0, 0), a, b);
  cout << "val: " << n.val << " a:" << n.a << " b:" << n.b << endl;
  pts.reserve(3);
  pts.push_back(n);
  vec3 dir = n.val * -1;
  while (true) {
    dir.norm();
    n = collision_vec(dir, a, b);
    cout << "val: " << n.val << " a:" << n.a << " b:" << n.b << endl;
    if (n.val.dot(dir) - pts.back().val.dot(dir) < 0.1) {
      break;
    }
    pts.push_back(n);
    process_simplex(pts, dir);
  }
}


epa_tri::epa_tri(simplex_pt &a, simplex_pt &b, simplex_pt &c) : a(a), b(b), c(c) {
  distSq = -1;
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
    if (it->distSq == -1) {
      vec3 v = it->norm * ((it->a.val * -1).dot(it->norm) / it->norm.dot(it->norm));
      it->distSq = v.lenSq();
    }
    if (it->distSq < min) {
      closest = it;
      min = it->distSq;
    }
  }
  return closest;
}

epa_tri epa(const collidable &one, const collidable &two, vector<simplex_pt> &pts) {

  simplex_pt &a = pts[3],
             &b = pts[2],
             &c = pts[1],
             &d = pts[0];

  cout << "a:" << a.a << b.a << c.a << d.a << endl;
  cout << "b:" << a.b << b.b << c.b << d.b << endl;
  cout << "val" << a.val << b.val << c.val << d.val << endl;

  std::list<epa_tri> tris;
  tris.push_back(epa_tri(a,b,c)); // abc
  tris.push_back(epa_tri(a,c,d)); // acd
  tris.push_back(epa_tri(a,d,b)); // adb
  tris.push_back(epa_tri(d,c,b)); // dcb

  list<epa_tri>::iterator t;
  while (true) {

    t = epa_min_dist(tris);

    cout << "1. " << t->a.a << t->b.a << t->c.a << t->norm << endl;
    cout << "2. " << t->a.b << t->b.b << t->c.b << t->norm << endl;
    simplex_pt p = collision_vec(t->norm, one, two);
    double d = p.val.dot(t->norm);
//    cout << distance<list<epa_tri>::iterator>(tris.begin(), tris.end()) << " - " << d << " dd " << (d*d) << " sq " << t->distSq << " dif " << (d*d - t->distSq) << endl;
    if (d*d - t->distSq < 0.001) {
      break;
    }
    else {
      simplex_pt v1 = t->a,
           v2 = t->b,
           v3 = t->c;
      cout << "Removing " << t->a.val << t->b.val << t->c.val << t->norm << " " << t->distSq << endl;
      tris.erase(t);

      cout << "Adding " << p.val << v1.val << v2.val << endl;
      cout << "Adding " << p.val << v2.val << v3.val << endl;
      cout << "Adding " << p.val << v3.val << v1.val << endl;
      tris.push_back(epa_tri(p, v1, v2));
      tris.push_back(epa_tri(p, v2, v3));
      tris.push_back(epa_tri(p, v3, v1));
    }
  }

  return *t;
}

typedef pair<vec3,vec3> edge;

vec3 find_intersection(edge &one, edge &two) {
  vec3 v1 = one.second - one.first,
       v2 = two.second - two.first;

  double l = ((two.first - one.first) * v2).len() / (v1*v2).len();

  return one.first + v1 * l;
}

list<edge> calculate_overlap(list<edge> a_edges, list<edge> b_edges, const vec3 &n) {

  for (list<edge>::iterator it = a_edges.begin(); it != a_edges.end(); it++) {
    vec3 left = it->first,
          right = it->second,
          norm = (right-left)*n;

//      cout << "OUTER: " << left << right << norm << endl;

    int count = 0;
    for (list<edge>::iterator it2 = b_edges.begin(); it2 != b_edges.end(); it2++) {
      vec3 left2 = it2->first,
            right2 = it2->second,
            b_norm = (right2 - left2) * n;

      count += 1;
//        cout << "INNER: " << left2 << right2 << b_norm << endl;
      
      bool l = norm.dot(left - left2) > 0;
      bool r = norm.dot(left - right2) > 0;
      if (!l && !r) {

//          cout << "Drop" << it2->first << it2->second << endl;
        it2 = --b_edges.erase(it2);

        continue;
      }
      count -= 1;

      bool overlap = (l ^ r);
      l = (left2 - left).dot(b_norm) > 0;
      r = (left2 - right).dot(b_norm) > 0;

      if (overlap && l^r) {
//          cout << "POP" << endl;

//          cout << "Int: " << left << right << '-' << left2 << right2 << endl;

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

    if (count == 0) {
//        cout << "Remove " << it->first << it->second << endl;
      it = --a_edges.erase(it);
    }
  }

  list<edge> edges = a_edges;
  edges.insert(edges.end(), b_edges.begin(),b_edges.end());

  for (list<edge>::iterator it = a_edges.begin(); it != a_edges.end(); it++) {
    cout << "A: " << it->first << " " << it->second << endl;
  }
  for (list<edge>::iterator it = b_edges.begin(); it != b_edges.end(); it++) {
    cout << "B: " << it->first << " " << it->second << endl;
  }

  return edges;
}



bool contact_points(const collidable &a, const collidable &b, list<vec3> &a_pts, list<vec3> &b_pts, vec3 &sep) {
  

  vector<simplex_pt> sim;
  vec3 sep_axis;
  if (!collide(a,b,sim, sep_axis)) return false;

  epa_tri t = epa(a, b, sim);
  vec3 n = t.norm;
  sep = n;
  vec3 perp = n*vec3(1.12345, 0.6543, 0.987564);
  perp *= 0.1 / perp.len();
  list<vec3> cpts = collision_points(a, n, perp, t.a.a, 10);
  vec3 inv = n*-1;
  list<vec3> cpts2 = collision_points(b, inv, perp, t.a.b, 10);

  cout << distance(cpts.begin(), cpts.end()) << " -- " << distance(cpts2.begin(), cpts2.end()) << endl;


  list<edge> a_edges;
  a_edges.push_back(edge(t.a.a + cpts.back(), t.a.a + cpts.front()));
  for (list<vec3>::iterator it = cpts.begin(); it != cpts.end(); it++) {
//    cout << "CA" << *it << endl;
    list<vec3>::iterator tmp = it;
    tmp++;
    if (tmp != cpts.end()) {
      a_edges.push_back(edge(t.a.a + *it, t.a.a + *tmp));
    }
  }
  


  list<edge> b_edges;
  b_edges.push_back(edge(t.a.b + cpts2.back(), t.a.b + cpts2.front()));
  for (list<vec3>::iterator it = cpts2.begin(); it != cpts2.end(); it++) {
//    cout << "CB" << *it << endl;
    list<vec3>::iterator tmp = it;
    tmp++;
    if (tmp != cpts2.end()) {
      b_edges.push_back(edge(t.a.b + *it, t.a.b + *tmp));
    }
  }

  // Swap B coords to find winding direction to be same as 'A'.
  for (list<edge>::iterator it = b_edges.begin(); it != b_edges.end(); it++) {
    swap(it->first, it->second);
  }


  list<edge> over = calculate_overlap(a_edges, b_edges, n);

  for (list<edge>::iterator it = over.begin(); it != over.end(); it++) {
    a_pts.push_back(it->first);
    b_pts.push_back(it->first);
  }

  return true;
}
