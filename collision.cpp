
#include <limits>
#include <iostream>
#include "collision.h"

using std::cout;
using std::cerr;
using std::endl;

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
  cerr << "one: " << one << " two: " << two << " dir: " << dir << " sum: " << s << endl;
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
      if (a.lenSq() == 0) {
        return true;
      }
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

      if (v.lenSq() == 0) {
        return true;
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

bool collision_point(const collidable &a, const collidable &b, vec3 &ap, vec3 &bp, vec3 &adir, vec3 &bdir) {
  std::vector<simplex_pt> pts;
  closest_simplex(a, b, pts);

  cout << "Simplex Size:" << pts.size() << endl;

  switch (pts.size()) {
    case 1: // 2 points
      cout << "pts: " << pts[0].val << pts[0].a << pts[0].b << endl;
      // pt-pt collision
      // read 2 pts from simplex
      ap = pts[0].a;
      bp = pts[0].b;
      adir = vec3();
      bdir = vec3();
      break;
    case 2: {// 3 or 4 points
      int count = 4;
      if (pts[0].a == pts[1].a) count -= 1;
      if (pts[0].a == pts[1].b) count -= 1;
      if (pts[0].b == pts[1].a) count -= 1;
      if (pts[0].b == pts[1].b) count -= 1;

      cout << "pts: " << pts[0].val << pts[0].a << pts[0].b << endl;
      cout << "     " << pts[1].val << pts[1].a << pts[1].b << endl;

      cout << "1-simplex";

      if (count == 4) { // edge-edge collision
        cout << ".1";
        vec3 a_diff = pts[1].a - pts[0].a;
        vec3 b_diff = pts[1].b - pts[0].b;
        if (a_diff.lenSq() < b_diff.lenSq()) {
          cout << ".1";
          ap = (pts[1].a + pts[0].a) * (1.0 / 2);
          vec3 v = (ap-pts[0].b);
          bp = b_diff * (v.dot(b_diff)/b_diff.dot(b_diff));
        }
        else {
          cout << ".2";
          bp = (pts[1].b + pts[0].b) * (1.0 / 2);
          vec3 v = (bp-pts[0].a);
          ap = a_diff * (v.dot(a_diff)/a_diff.dot(a_diff));
        }
      }
      else if (count == 3) { // edge-edge or edge-pt
        cout << ".2";
        if (pts[0].a == pts[1].a) { //
          cout << ".1";
          
          vec3 b_diff = pts[1].b - pts[0].b;
          vec3 pt = a.collision_point(b_diff);
          vec3 a_diff = pt - pts[0].a;
          if ((a_diff*b_diff).lenSq() < 0.1) { // edge-edge
            cout << ".1";
            vec3 v = (pts[1].a - pts[1].b) * (1.0 / 2);
            bp = pts[1].b + b_diff * (v.dot(b_diff)/b_diff.dot(b_diff));
            ap = pts[0].a - a_diff * (v.dot(a_diff)/a_diff.dot(a_diff));
          }
          else {
            cout << ".2";
            pt = a.collision_point(b_diff * -1);
            vec3 a_diff = pt-pts[0].a;
            if ((a_diff*b_diff).lenSq() < 0.1) { // edge-edge
              cout << ".1";
              vec3 v = (pts[0].a - pts[0].b) * (1.0 / 2);
              bp = pts[0].b + (b_diff * (v.dot(b_diff)/b_diff.dot(b_diff)));
              ap = pts[0].a + (a_diff * -1) * (v.dot(a_diff)/a_diff.dot(a_diff));
            }
            else { // edge-pt
              cout << ".2";
              vec3 v = (pts[0].a - pts[0].b);
              bp = pts[0].b + b_diff * (v.dot(b_diff)/b_diff.dot(b_diff));
              ap = pts[0].a;
            }
          }
        }
        else if (pts[0].b == pts[1].b) {
          cout << ".2";
          vec3 a_diff = pts[1].a - pts[0].a;
          vec3 pt = b.collision_point(a_diff);
          vec3 b_diff = pt - pts[0].b;
          if ((a_diff*b_diff).lenSq() < 0.1) { // edge-edge
            cout << ".1";
            vec3 v = (pts[1].b - pts[1].a) * (1.0 / 2);
            ap = pts[1].a + a_diff * (v.dot(a_diff)/a_diff.dot(a_diff));
            bp = pts[0].b - b_diff * (v.dot(b_diff)/b_diff.dot(b_diff));
          }
          else {
            cout << ".2";
            pt = b.collision_point(a_diff * -1);
            vec3 b_diff = pt-pts[0].b;
            if ((a_diff*b_diff).lenSq() < 0.1) { // edge-edge
              cout << ".1";
              vec3 v = (pts[0].b - pts[0].a) * (1.0 / 2);
              ap = pts[0].a + (a_diff * (v.dot(a_diff)/a_diff.dot(a_diff)));
              bp = pts[0].b + (b_diff * -1) * (v.dot(b_diff)/b_diff.dot(b_diff));
            }
            else { // edge-pt
              cout << ".2";
              vec3 v = (pts[0].b - pts[0].a);
              ap = pts[0].a + a_diff * (v.dot(a_diff)/a_diff.dot(a_diff));
              bp = pts[0].b;
            }
          }
        }
        else {
          cerr << "Unexpected collision type 1" << endl;
        }
      }
      else {
        cerr << "Unexpected collision type 2" << endl;
      }
      cout << endl;
      // else check if neighbors of odd-out pt are on same line as 2 main points
      break;
    }
    case 3: // 4, 5 or 6 points
      cout << "pts: " << pts[0].val << pts[0].a << pts[0].b << endl;
      cout << "     " << pts[1].val << pts[1].a << pts[1].b << endl;
      cout << "     " << pts[2].val << pts[2].a << pts[2].b << endl;
      // if 3, pt-face
      // if 6 face-face
      //
      int count = 6;
      if (pts[0].a == pts[1].a) count -= 1;
      if (pts[0].a == pts[1].b) count -= 1;
      if (pts[0].b == pts[1].a) count -= 1;
      if (pts[0].b == pts[1].b) count -= 1;



      break;
  }
  return true;
}













