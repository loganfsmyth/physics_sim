
#include <vector>
#include "collision.h"

#include <iostream>
using namespace std;

vec3 collision_vec(vec3 dir, const collidable &a, const collidable &b) {
  vec3 one = a.collision_point(dir);
  vec3 two = b.collision_point(dir*-1);
  vec3 s = one-two;
  
//  cerr << "one: " << one << " two: " << two << " dir: " << dir << " sum: " << s << endl;
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
      cerr << "ab:" << ab << " = " << "a0:" <<a0 << " = v:" << v << endl;

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
      

      cerr << "triangle: a: " << a << " b: " << b << " c: " << c << endl;
//      cerr << "\t\tabc: " << abc << " ab: " << ab << " ac: " << ac << " a0: " << a0 << endl;
      cerr << "triangle";
      if ((abc*ac).dot(a0) > 0) { // ac or ab edge or a corner
        cerr << ".1";
        if (ac.dot(a0) > 0) { // ac edge
          cerr << ".1";
          // pts => c,a
          pts[1] = a;
          pts.pop_back();
          dir = ac * a0 * ac;
        }
        else {
          cerr << ".2";
          if (ab.dot(a0) > 0) { // ab edge
            cerr << ".1";
            // pts => b,a
            pts[0] = b;
            pts[1] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            cerr << ".2";
            // pts => a
            pts[0] = a;
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
      }
      else {
        cerr << ".2";
        if ((ab*abc).dot(a0) > 0) { // ab edge or a corner
          cerr << ".1";
          if (ab.dot(a0) > 0) { // ab edge
            cerr << ".1";
            // pts => b,a
            pts[0] = b;
            pts[1] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            cerr << ".2";
            // pts => a
            pts[0] = a;
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
        else { // inside triangle, above or below
          cerr << "-2";
          double v = abc.dot(a0);
          if (v == 0) {
            dir = a0;
            cerr << endl;
            return true;
          }
          else if (v > 0) { // above  points 
            cerr << ".1";
            // pts => c,b,a
            dir = abc;
          }
          else { // below
            cerr << ".2";
            // pts => b,c,a
            vec3 tmp = c;
            pts[0] = b;
            pts[1] = tmp;
            dir = abc * -1;
          }
        }
      }
      cerr << endl;
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

      cerr << "ab: " << ab << " ac: " << ac << " ad: " << ad << " a0: " << a0 << endl;
      cerr << "abc: " << abc << " abd: " << abd << " acd: " << acd << endl;

      cerr << "tetra";

      if (abc.dot(a0) > 0) { // bc edges/corners excluded by prior info
        cerr << ".1";
        if ((ab * abc).dot(a0) > 0) { // abd face, ab edge or a corner
          cerr << ".1";
          if (ab.dot(a0) > 0) { // ab edge or abd face
            cerr << ".1";
            if ((ab*abd).dot(a0) > 0) { // ab edge
              cerr << ".1";
              // pts => b,a
              pts[0] = b;
              pts[1] = a;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // abd face
              cerr << ".2";
              // pts => d,b,a
              pts[1] = b;
              pts[2] = a;
              pts.pop_back();
              dir = abd;
            }
          }
          else {
            cerr << ".2";
            if (ad.dot(a0) < 0) {
              cerr << ".1";
              if (ac.dot(a0) < 0) { // a corner
                cerr << ".1";
                // pts => a
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // acd face
                cerr << ".2";
                // pts => d,c,a
                pts[2] = a;
                pts.pop_back();
                dir = acd;
              }
            }
            else { // abd face
              cerr << ".2";
              // pts => d,b,a
              pts[1] = b;
              pts[2] = a;
              pts.pop_back();
              dir = abd;
            }
          }
        }
        else {
          cerr << ".2";
          if ((abc*ac).dot(a0) > 0) { // ac edge or a corner
            cerr << ".1";
            if (ac.dot(a0) > 0) { // ac edge
              cerr << ".1";
              if ((ac*acd).dot(a0) > 0) { // ac edge
                cerr << ".1";
                // pts => c,a
                pts[0] = c;
                pts[1] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // acd face
                cerr << ".2";
                // pts => d,c,a
                pts[2] = a;
                pts.pop_back();
                dir = acd;
              }
            }
            else {
              cerr << ".2";
              if (ad.dot(a0) < 0) { // a corner
                cerr << ".1";
                // pts => a
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // ad edge
                cerr << ".2";
                // pts => d,a
                pts[1] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ad*a0*ad;
              }
            }
          }
          else { // abc face
            cerr << ".2";
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
        cerr << ".2";
        if (abd.dot(a0) < 0) { // abd plane, bd edges/corders excluded by prior info
          cerr << ".1";
          if ((ab*abd).dot(a0) > 0) { // ab edge or a corner
            cerr << ".1";
            if (ab.dot(a0) > 0) { // ab edge
              cerr << ".1";
              // pts => b,a
              pts[0] = b;
              pts[1] = a;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // a corner
              cerr << ".2";
              // pts => a
              pts[0] = a;
              pts.pop_back();
              pts.pop_back();
              pts.pop_back();
              dir = a0;
            }
          }
          else {
            cerr << ".2";
            if ((abd*ad).dot(a0) > 0) { // ad edge or a corner
              cerr << ".1";
              if (ad.dot(a0) > 0) {
                cerr << ".1";
                if ((acd*ad).dot(a0) > 0) { // ad edge
                  cerr << ".1";
                  // pts => d,a
                  pts[1] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // acd face
                  cerr << ".2";
                  // pts => d,c,a
                  pts[2] = a;
                  pts.pop_back();
                  dir = acd;
                }
              }
              else { // a corner
                cerr << ".2";
                // pts => a
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else { // abd face
              cerr << ".2";
              // pts => d,b,a
              pts[1] = b;
              pts[2] = a;
              pts.pop_back();
              dir = abd * -1;
            }
          }
        }
        else { // abd face out
          cerr << ".2";
          if (acd.dot(a0) > 0) { // acd plane, cd edges/corners excluded by prior info
            cerr << ".1";
            if ((ac*acd).dot(a0) > 0) { // ac edge or a corner
              cerr << ".1";
              if (ac.dot(a0) > 0) { // ac edge
                cerr << ".1";
                // pts => c,a
                pts[0] = c;
                pts[1] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // a corner
                cerr << ".2";
                // pts => a
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else {
              cerr << ".2";
              if ((acd*ad).dot(a0) > 0) { // ad edge or a corner
                cerr << ".1";
                if (ad.dot(a0) > 0) { // ad edge
                  cerr << ".1";
                  // pts => d,a
                  pts[1] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // a corner
                  cerr << ".2";
                  // pts => a
                  pts[0] = a;
                  pts.pop_back();
                  pts.pop_back();
                  pts.pop_back();
                  dir = a0;
                }
              }
              else { // acd face
                cerr << ".2";
                // pts => d,b,a
                pts[1] = c;
                pts[2] = a;
                pts.pop_back();
                dir = acd;
              }
            }
          }
          else {
            cerr << ".2";
            cerr << endl;
            return true;
          }
        }
      }
      cerr << endl;
      break;
    }
  }


  return false;
}



bool collide(const collidable &a, const collidable &b) {
  std::vector<vec3> pts;
  vec3 n,
       p = collision_vec(vec3(1.0f, 0.0f, 0.0f), a, b);

//  cerr << "p: " << p << endl;

  pts.reserve(4);
  pts.push_back(p);
  p *= -1;
  while (true) {
//  for (int i = 0; i < 5; i++) {
    n = collision_vec(p, a, b);
    cerr << "== n: " << n << " dir: " << p << endl;
    cerr << n.dot(p) << endl;

    if (n.dot(p) < 0) return false;
    pts.push_back(n);
    if (process_simplex(pts, p)) return true;
//    cerr << " = p: " << p << endl;
  }
  return false;
}


