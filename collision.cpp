
#include <vector>
#include "collision.h"

#include <iostream>
using namespace std;

#ifndef COLLISION_DEBUG
#define COLLISION_DEBUG 0
#endif


vec3 collision_vec(vec3 dir, const collidable &a, const collidable &b) {
  vec3 one = a.collision_point(dir);
  vec3 two = b.collision_point(dir*-1);
  vec3 s = one-two;
  
#if COLLISION_DEBUG
  cerr << "one: " << one << " two: " << two << " dir: " << dir << " sum: " << s << endl;
#endif
  return s;
}

bool process_simplex(std::vector<vec3> &pts, vec3 &dir) {
  switch (pts.size()) {
    case 0:
      break;
    case 1: {
      vec3 &a = pts[0];
      if (a.lenSq() == 0) {
        return true;
      }
      break;
    }
    case 2: {
      vec3 &a = pts[1];
      vec3 &b = pts[0];
      vec3 ab = b - a;
      vec3 a0 = a * -1;

      double dist = ab.dot(a0);

      vec3 v = (ab*a0);
#if COLLISION_DEBUG
      cerr << "ab:" << ab << " = " << "a0:" <<a0 << " = v:" << v << endl;
#endif

      if (v.lenSq() == 0) {
        return true;
      }
      else if (dist > 0) {
        dir = (v * ab);
      }
      else {
        pts[0] = a;
        pts.pop_back();
        dir = a0;
      }
      break;
    }
    case 3: {
      vec3 &a = pts[2];
      vec3 &b = pts[1];
      vec3 &c = pts[0];
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
          pts[1] = a;
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
            pts[0] = b;
            pts[1] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => a
            pts[0] = a;
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
            pts[0] = b;
            pts[1] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
#if COLLISION_DEBUG
            cerr << ".2";
#endif
            // pts => a
            pts[0] = a;
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
            vec3 tmp = c;
            pts[0] = b;
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
      vec3 &a = pts[3];
      vec3 &b = pts[2];
      vec3 &c = pts[1];
      vec3 &d = pts[0];
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 ad = d - a;
      vec3 a0 = a * -1;
      vec3 abc = (ab*ac);
      vec3 abd = (ab*ad);
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
              pts[0] = b;
              pts[1] = a;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // abd face
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => b,d,a, wound opposite
              vec3 tmp = d;
              pts[0] = b;
              pts[1] = tmp;
              pts[2] = a;
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
                pts[0] = a;
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
                pts[2] = a;
                pts.pop_back();
                dir = acd;
              }
            }
            else { // abd face
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => b,d,a, wound opposite
              vec3 tmp = d;
              pts[0] = b;
              pts[1] = tmp;
              pts[2] = a;
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
                pts[0] = c;
                pts[1] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // acd face
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => d,c,a
                pts[2] = a;
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
                pts[0] = a;
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
                pts[1] = a;
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
            pts[0] = c;
            pts[1] = b;
            pts[2] = a;
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
              pts[0] = b;
              pts[1] = a;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // a corner
#if COLLISION_DEBUG
              cerr << ".2";
#endif
              // pts => a
              pts[0] = a;
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
                  pts[1] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // acd face
#if COLLISION_DEBUG
                  cerr << ".2";
#endif
                  // pts => d,c,a
                  pts[2] = a;
                  pts.pop_back();
                  dir = acd;
                }
              }
              else { // a corner
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => a
                pts[0] = a;
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
              vec3 tmp = d;
              pts[0] = b;
              pts[1] = tmp;
              pts[2] = a;
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
                pts[0] = c;
                pts[1] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // a corner
#if COLLISION_DEBUG
                cerr << ".2";
#endif
                // pts => a
                pts[0] = a;
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
                  pts[1] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // a corner
#if COLLISION_DEBUG
                  cerr << ".2";
#endif
                  // pts => a
                  pts[0] = a;
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
                // pts => d,b,a
                pts[1] = c;
                pts[2] = a;
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



bool collide(const collidable &a, const collidable &b) {
  std::vector<vec3> pts;
  vec3 n,
       p = collision_vec(vec3(1.0f, 0.0f, 0.0f), a, b);

  pts.reserve(4);
  pts.push_back(p);
  p *= -1;
  while (true) {
    n = collision_vec(p, a, b);
#if COLLISION_DEBUG
    cerr << "== n: " << n << " dir: " << p << endl;
    cerr << n.dot(p) << endl;
#endif
    if (n.dot(p) < 0) return false;
    pts.push_back(n);
    if (process_simplex(pts, p)) return true;
  }
  return false;
}


