
#include <vector>
#include "collision.h"


vec3 collision_vec(vec3 dir, const collidable &a, const collidable &b) {
  return a.collision_point(dir) + b.collision_point(dir*-1);
}

bool process_simplex(std::vector<vec3> &pts, vec3 &dir) {
  switch (pts.size()) {
    case 0:
    case 1:
      break;
    case 2: {
      vec3 &a = pts[1];
      vec3 &b = pts[0];
      vec3 ab = b - a;
      vec3 a0 = a * -1;

      if (ab.dot(a0) > 0) {
        dir = (ab * a0 * ab);
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
      vec3 &c = pts[0]
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 a0 = a * -1;
      vec3 abc = (ab*ac);
      
      if ((abc*ac).dot(a0) > 0) { // ac or ab edge or a corner
        if (ac.dot(a0) > 0) { // ac edge
          pts[1] = c;
          pts[0] = a;
          pts.pop_back();
          dir = ac * a0 * ac;
        }
        else {
          if (ab * a0 > 0) { // ab edge
            pts[0] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            pts[0] = a;
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
      }
      else {
        if ((ab*abc).dot(a0) > 0) { // ab edge or a corner
          if (ab * a0 > 0) { // ab edge
            pts[0] = a;
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            pts[0] = a;
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
        else { // inside triangle, above or below
          vec3 ctmp = c;
          pts[0] = a;
          if (abc.dot(a0) > 0) { // above
            pts[2] = ctmp;
            dir = abc;
          }
          else { // below
            pts[2] = b;
            pts[1] = ctmp;
            dir = abc * -1;
          }
        }
      }
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

      if (abc.dot(a0) > 0) { // bc edges/corners excluded by prior info
        if ((abc * ab).dot(a0) > 0) { // abd face, ab edge or a corner
          if (ab.dot(a0) > 0) { // ab edge or abd face
            if ((abd*ab).dot(a0) > 0) { // ab edge
              pts[0] = a;
              pts[1] = b;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // abd face
              pts[2] = d;
              pts[1] = b;
              pts[0] = a;
              pts.pop_back();
              dir = abd;
            }
          }
          else {
            if (ad.dot(a0) > 0) {
              if (ac.dot(a0) > 0) {
                // a corner
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else {
                // acd face
                pts[1] = c;
                pts[2] = d;
                pts[0] = a;
                pts.pop_back();
                dir = acd;
              }
            }
            else {
              // abd face
              pts[1] = b;
              pts[2] = d;
              pts[0] = a;
              pts.pop_back();
              dir = abd;
            }
          }
        }
        else {
          if ((abc*ac).dot(a0) > 0) { // ac edge or a corner
            if (ac.dot(a0) > 0) { // ac edge
              if ((acd*ac).dot(a0) > 0) {
                // ac edge
                pts[0] = a;
                pts[1] = c;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else {
                // acd face
                pts[2] = d;
                pts[1] = c;
                pts[0] = a;
                pts.pop_back();
                dir = acd;
              }
            }
            else { // a corner
              if (ad.dot(a0) > 0) {
                // a corner
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else {
                // ad edge
                pts[1] = d;
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                dir = ad*a0*ad;
              }
            }
          }
          else { // abc face
            pts[0] = a;
            pts[3] = c;
            pts[1] = b;
            pts[2] = pts[3];
            pts.pop_back();
            dir = abc;
          }
        }
      }
      else { // abc face out
        if (abd.dot(a0) > 0) { // abd plane, bd edges/corders excluded by prior info
          if ((abd*ab).dot(a0) > 0) { // ab edge or a corner
            if (ab.dot(a0) > 0) { // ab edge
              pts[0] = a;
              pts[1] = b;
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // a corner
              pts[0] = a;
              pts.pop_back();
              pts.pop_back();
              pts.pop_back();
              dir = a0;
            }
          }
          else {
            if ((abd*ad).dot(a0) > 0) { // ad edge or a corner
              if (ad.dot(a0) > 0) {
                if ((acd*ad).dot(a0) > 0) {
                  // ad edge
                  pts[1] = d;
                  pts[0] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else {
                  // acd face
                  pts[0] = a;
                  pts[3] = pts[2];
                  pts[2] = d;
                  pts[1] = pts[3];
                  pts.pop_back();
                  dir = acd;
                }
              }
              else { // a corner
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else { // abd face
              pts[1] = b;
              pts[2] = d;
              pts[0] = a;
              pts.pop_back();
              dir = abd;
            }
          }
        }
        else { // abd face out
          if (acd.dot(a0) > 0) { // acd plane, cd edges/corners excluded by prior info
            if ((acd*ac).dot(a0) > 0) { // ac edge or a corner
              if (ac.dot(a0) > 0) { // ac edge
                pts[0] = a;
                pts[1] = c;
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // a corner
                pts[0] = a;
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else {
              if ((acd*ad).dot(a0) > 0) { // ad edge or a corner
                if (ad.dot(a0) > 0) { // ad edge
                  pts[1] = d;
                  pts[0] = a;
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // a corner
                  pts[0] = a;
                  pts.pop_back();
                  pts.pop_back();
                  pts.pop_back();
                  dir = a0;
                }
              }
              else { // abd face
                pts[3] = d;
                pts[0] = a;
                pts[1] = b;
                pts[2] = pts[3];
                pts.pop_back();
                dir = abd;
              }
            }
          }
          else {
            return true;
          }
        }
      }
      break;
    }
  }

  return false;
}


bool collide(const collidable &a, const collidable &b) {
  std::vector<vec3> pts;
  vec3 n,
       p = collision_vec(vec3(0.0f, 0.0f, 0.0f), a, b);

  pts.reserve(4);
  pts[0] = p;
  p *= -1;
  while (true) {
    n = collision_vec(p, a, b);
    if (n.dot(p) < 0) return false;
    pts.push_back(n);
    if (process_simplex(pts, p)) return true;
  }
}


