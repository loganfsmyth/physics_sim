
#include <list>
#include <set>

#include <iostream>
#include "collision.h"
#include <cmath>
#include <cassert>

#include "chull.h"

//#define DBG(c) cerr << c

#ifndef DBG
#define DBG(c)
#endif

using std::cerr;
using std::cerr;
using std::endl;
using namespace std;

typedef pair<vec3,vec3> edge;

simplex_pt::simplex_pt(vec3 v, vec3 na, vec3 nb): val(v), a(na), b(nb) { }
simplex_pt::simplex_pt() { }
bool simplex_pt::operator==(const simplex_pt &p) {
  return a == p.a && b == p.b;
}

/**
 * Support function for GJK and EPA. Given direction and 2 objects, return simplex point.
 */
simplex_pt collision_vec(vec3 dir, const collidable &a, const collidable &b) {
  vec3 one = a.collision_point(dir);
  vec3 two = b.collision_point(dir*-1);
  vec3 s = one-two;
  
  simplex_pt val(s,one,two);
  return val;
}

/**
 * Given a current simplex, update contents based on location of origin,
 * and set 'dir' as vector pointing toward origin.
 */
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
      

      if ((abc*ac).dot(a0) > 0) { // ac or ab edge or a corner
        if (ac.dot(a0) > 0) { // ac edge
          // pts => c,a
          pts[1] = pts[2];
          pts.pop_back();
          dir = ac * a0 * ac;
        }
        else {
          if (ab.dot(a0) > 0) { // ab edge
            // pts => b,a
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            // pts => a
            pts[0] = pts[2];
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
      }
      else {
        if ((ab*abc).dot(a0) > 0) { // ab edge or a corner
          if (ab.dot(a0) > 0) { // ab edge
            // pts => b,a
            pts[0] = pts[1];
            pts[1] = pts[2];
            pts.pop_back();
            dir = ab * a0 * ab;
          }
          else { // a corner
            // pts => a
            pts[0] = pts[2];
            pts.pop_back();
            pts.pop_back();
            dir = a0;
          }
        }
        else { // inside triangle, above or below
          double v = abc.dot(a0);
          if (v == 0) {
            dir = a0;
            return true;
          }
          else if (v > 0) { // above  points 
            // pts => c,b,a
            dir = abc;
          }
          else { // below
            // pts => b,c,a
            simplex_pt tmp = pts[0];
            pts[0] = pts[1];
            pts[1] = tmp;
            dir = abc * -1;
          }
        }
      }
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

      DBG("tet.");
      if (abc.dot(a0) > 0) { // bc edges/corners excluded by prior info
        DBG("1.");
        if ((ab * abc).dot(a0) > 0) { // abd face, ab edge or a corner
          DBG("1.");
          if (ab.dot(a0) > 0) { // ab edge or abd face
            DBG("1.");
            if ((ab*abd).dot(a0) > 0) { // ab edge
              DBG("1.");
              // pts => b,a
              pts[0] = pts[2];
              pts[1] = pts[3];
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // abd face
              DBG("2.");
              // pts => b,d,a, wound opposite
              simplex_pt tmp = pts[0];
              pts[0] = pts[2];
              pts[1] = tmp;
              pts[2] = pts[3];
              pts.pop_back();
              dir = abd * -1;
            }
          }
          else { // TODO: Check all these conditions
            DBG("2.");
            if (ad.dot(a0) < 0) {
              DBG("1.");
              if (ac.dot(a0) < 0) { // a corner
                DBG("1.");
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // acd face
                DBG("2.");
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
            else { // abd face
              DBG("2.");
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
          DBG("2.");
          if ((abc*ac).dot(a0) > 0) { // ac edge or a corner
            DBG("1.");
            if (ac.dot(a0) > 0) { // ac edge
              DBG("1.");
              if ((ac*acd).dot(a0) > 0) { // ac edge
                DBG("1.");
                // pts => c,a
                pts[0] = pts[1];
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // acd face
                DBG("2.");
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
            else { // TODO Can this be abd face too?
              DBG("2.");
              if (ad.dot(a0) < 0) { // a corner
                DBG("1.");
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
              else { // ad edge
                DBG("2.");
                // pts => d,a
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ad*a0*ad;
              }
            }
          }
          else { // abc face
            DBG("2.");
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
        DBG("2.");
        if (abd.dot(a0) < 0) { // abd plane, bd edges/corders excluded by prior info
          DBG("1.");
          if ((ab*abd).dot(a0) > 0) { // ab edge or a corner
            DBG("1.");
            if (ab.dot(a0) > 0) { // ab edge
              DBG("1.");
              // pts => b,a
              pts[0] = pts[2];
              pts[1] = pts[3];
              pts.pop_back();
              pts.pop_back();
              dir = ab*a0*ab;
            }
            else { // a corner
              DBG("2.");
              // pts => a
              pts[0] = pts[3];
              pts.pop_back();
              pts.pop_back();
              pts.pop_back();
              dir = a0;
            }
          }
          else {
            DBG("2.");
            if ((abd*ad).dot(a0) > 0) { // ad edge or a corner
              DBG("1.");
              if (ad.dot(a0) > 0) {
                DBG("1.");
                if ((acd*ad).dot(a0) > 0) { // ad edge
                  DBG("1.");
                  // pts => d,a
                  pts[1] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // acd face
                  DBG("2.");
                  // pts => d,c,a
                  pts[2] = pts[3];
                  pts.pop_back();
                  dir = acd;
                }
              }
              else { // a corner
                DBG("2.");
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else { // abd face
              DBG("2.");
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
          DBG("2.");
          if (acd.dot(a0) > 0) { // acd plane, cd edges/corners excluded by prior info
            DBG("1.");
            if ((ac*acd).dot(a0) > 0) { // ac edge or a corner
              DBG("1.");
              if (ac.dot(a0) > 0) { // ac edge
                DBG("1.");
                // pts => c,a
                pts[0] = pts[1];
                pts[1] = pts[3];
                pts.pop_back();
                pts.pop_back();
                dir = ac*a0*ac;
              }
              else { // a corner
                DBG("2.");
                // pts => a
                pts[0] = pts[3];
                pts.pop_back();
                pts.pop_back();
                pts.pop_back();
                dir = a0;
              }
            }
            else {
              DBG("2.");
              if ((acd*ad).dot(a0) > 0) { // ad edge or a corner
                DBG("1.");
                if (ad.dot(a0) > 0) { // ad edge
                  DBG("1.");
                  // pts => d,a
                  pts[1] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  dir = ad*a0*ad;
                }
                else { // a corner
                  DBG("2.");
                  // pts => a
                  pts[0] = pts[3];
                  pts.pop_back();
                  pts.pop_back();
                  pts.pop_back();
                  dir = a0;
                }
              }
              else { // acd face
                DBG("2.");
                // pts => d,c,a
                pts[2] = pts[3];
                pts.pop_back();
                dir = acd;
              }
            }
          }
          else {
            DBG("2.");
            DBG(endl);
            return true;
          }
        }
      }
      break;
    }
  }
  DBG(endl);

  return false;
}

/**
 * Implementation of GJK algorithm.
 *
 * Given 2 'collidable' objects, returns true if they are colliding.
 * Using 2nd form, you can also extract the resulting simplex and normals.
 */
bool collide(const collidable &a, const collidable &b) {
  std::vector<simplex_pt> pts;
  vec3 dir;
  return collide(a, b, pts, dir);
}
bool collide(const collidable &a, const collidable &b, std::vector<simplex_pt> &pts, vec3 &dir) {
  const int max_iterations = 8;
  simplex_pt n,
             pt = collision_vec(vec3(1.0f, 0.0f, 0.0f), a, b);

  pts.reserve(4);
  pts.push_back(pt);
  dir = pt.val * -1;
//  cout << "Collide" << endl;
  int i;
  for (i = 0; i < max_iterations; i++) {
    n = collision_vec(dir, a, b);
  /*
    if (pts.size() == 3) {
      cout << "Vals: " << pts[0].val << '\t'   << pts[1].val << '\t' << pts[2].val << endl;
      cout << "A:    " << pts[0].a   << '\t'   << pts[1].a   << '\t' << pts[2].a   << endl;
      cout << "B:    " << pts[0].b   << "\t\t" << pts[1].b   << '\t' << pts[2].b   << endl;
      cout << "New:  " << n.val << "\t" << n.a << "\t" << n.b << " dir: " << dir << endl;
      cout << endl;
    }
    else {
      cout << pts.size() << endl;
    }
  */
    if (n.val.dot(dir) < 0) return false;
    pts.push_back(n);
    if (process_simplex(pts, dir)) return true;
  }

  if (i == max_iterations) {
    throw string("GJK hit max iterations");
  }

  return false;
}


/**
 * Helper to rotate a vector around a axis by some angle.
 */
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

/**
 * Perturbation method for finding all collision points with a plane.
 *
 * Given an object, normal, and a point on the surface, among others,
 * perturbs the normal vector to calculate points that end up on the
 * opposite side of the plane.
 */
list<vec3> collision_points(const collidable &a, const vec3 &n, vec3 perp, const vec3 &pt, int samples) {
  double angle = 2*3.1415926535 / samples;
  list<vec3> pts;
  for (int i = 0; i < samples; i++) {
    // Find new vertex in direction of perturbed normal.
    vec3 to_vert = a.collision_point(n + perp) - pt;

    // Project vertex onto plane.
    vec3 on_plane = to_vert - n * n.dot(to_vert);
    bool found = false;

    // Only save new points.
    for (list<vec3>::iterator it = pts.begin(); it != pts.end(); it++) {
      if (on_plane == *it) found = true;
    }

    if (!found) {
      pts.push_back(on_plane);
    }

    // Shift perturbation vector around normal.
    rotateVec(perp, angle, n);
  }

  return pts;
}

epa_tri::epa_tri(simplex_pt &a, simplex_pt &b, simplex_pt &c) : a(a), b(b), c(c) {
  dist = -1;
  ab = b.val-a.val;
  ac = c.val-a.val;
  bc = c.val-b.val;
  norm = ab*ac;
  norm.norm();
}

/**
 * Implementation of the EPA (Expanding Polytop Algorithm).
 */
epa_tri epa(const collidable &one, const collidable &two, vector<simplex_pt> &pts) {

  const int max_iterations = 40;


  simplex_pt &a = pts[3],
             &b = pts[2],
             &c = pts[1],
             &d = pts[0];

//  cout << "===================================" << endl;
  // Create basic convex hull from GJK simplex.
  chull h(a, b, c, d);

  int fid;
  // Expand hull until no new points can be found.
  int i;
  for (i = 0; i < max_iterations; i++) {
    pair<double,int> face = h.closestFace();
    fid = face.second;
    vec3 norm = h.fNorm(fid);

    simplex_pt p = collision_vec(norm, one, two);
    double d = p.val.dot(norm);
//    cout << d << " " << face.first << endl;
    if (d - face.first < 0.005) {
      break;
    }
    h.add_pt(p);
  }

  if (i == max_iterations) {
    throw string("EPA Max iterations");
  }

  // Build an EPA triangle structure from hull face.
  // @TODO This can be cleaned up. Left over from earlier implementation.
  return h.getTri(fid);
}

/**
 * Given 2 edges in 3d space, find intersection.
 * This assumes they intersect, if they don't, then the point found will
 * be incorrect.
 */
vec3 find_intersection(edge &one, edge &two) {
  vec3 v1 = one.second - one.first,
       v2 = two.second - two.first;

  double l = ((two.first - one.first) * v2).len() / (v1*v2).len();

  return one.first + v1 * l;
}

/**
 * Given two polygons in 3d space, project onto plane and find overlaping areas.
 */
list<edge> calculate_overlap(list<edge> small, list<edge> large, const vec3 &n) {

  int small_len = distance(small.begin(),small.end());
  int large_len = distance(large.begin(),large.end());

  if (small_len > large_len) {
    swap(small,large);
    swap(small_len,large_len);
  }

  if (large_len >= 3) {
    if (small_len >= 3) { // plane-plane

      // Go through all edge-edge comparisons and trim/remove edges where needed.
      for (list<edge>::iterator it = large.begin(); it != large.end(); it++) {
        vec3 left = it->first,
              right = it->second,
              norm = (right-left)*n;

        int count = 0;
        bool rem = false;
        for (list<edge>::iterator it2 = small.begin(); it2 != small.end(); it2++) {
          vec3 left2 = it2->first,
                right2 = it2->second,
                b_norm = (right2 - left2) * n;

          // Remove edges if they are fully outside other shape.
          bool l = norm.dot(left - left2) > 0;
          bool r = norm.dot(left - right2) > 0;
          if (!l && !r) {
            it2 = --small.erase(it2);
            rem = true;
            continue;
          }


          bool overlap = (l ^ r);
          l = (left2 - left).dot(b_norm) > 0;
          r = (left2 - right).dot(b_norm) > 0;
          if (overlap && l^r) {
            // If they overlap fully, find intersection and crop lines.
            // This isn't quite right, because find_inter assumes they
            // fully intersect in 3d space but good enough for now.
            // @TODO
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
        if (!rem) {
          it = --large.erase(it);
        }
      }
    }
    else if (small_len == 2) { // plane-edge
      edge e(small.front());
      vec3 &start = e.first,
           &end = e.second,
           norm = (start-end)*n;

      // Go through plane edges and crop edge to fit inside plane.
      for (list<edge>::iterator it = large.begin(); it != large.end(); it++) {
        vec3 &left = it->first,
             &right = it->second,
             inner_norm = (right-left)*n;
        
        bool l = norm.dot(start-left) > 0;
        bool r = norm.dot(start-right) > 0;
        
        bool l2 = inner_norm.dot(start-left) > 0;
        bool r2 = inner_norm.dot(end-left) > 0;
        
        if (l ^ r && l2 ^ r2) {
          // @TODO find_intersection assumes full intersect
          vec3 in = find_intersection(*it, e);
          if ((start - in).dot(inner_norm) > 0) {
            start = in;
          }
          else {
            end = in;
          }
        }
      }

      list<edge> edges;
      edges.push_back(edge(start,end));
      edges.push_back(edge(end,start));
      return edges;
    }
    else if (small_len == 1) { // plane-pt
      return small;
    }
    else {
      cerr << "Object has no contact points" << endl;
    }
  }
  else if (large_len == 2) {
    if (small_len == 2) { // line-line
      edge e1(small.front());
      edge e2(large.front());
      list<edge> edges;
      if (false && ((e1.first-e1.second)*(e2.first-e2.second)).lenSq() == 0) {
        // Ignoring this case for now.
        // @TODO
        cerr << "Line-line" << endl;
      }
      else {
        // Return new edge with just collision points.
        vec3 in = find_intersection(e1, e2);
        edges.push_back(edge(in,in));
      }
      return edges;
    }
    else if (small_len == 1) { // line-pt
      // Return same pt.
      return small;
    }
    else {
      cout << "Object has no contact points" << endl;
    }
  }
  else if (large_len == 1) {
    if (small_len == 1) { // pt-pt
      // Return same point.
      return small;
    }
    else {
      cout << "Object has no contact points" << endl;
    }
  }
  else {
      cout << "Object has no contact points" << endl;

  }

  // For general case, just merge edges still in large and small.
  list<edge> edges(large);
  edges.insert(edges.end(), small.begin(), small.end());
  return edges;
}


/**
 * Find contact points between two collidable objects.
 * Returns true if they collide.
 * a_pts/b_pts contain the same values currently, the points that contact.
 * sep is the surface normal for the collision.
 */
bool contact_points(collidable &a, collidable &b, list<vec3> &a_pts, list<vec3> &b_pts, vec3 &sep) {
  
  // Check if objects collide using GJK
  vector<simplex_pt> sim;
  vec3 sep_axis;
  if (!collide(a,b,sim, sep_axis)) return false;

  // Find collision normal using EPA.
  epa_tri t = epa(a, b, sim);
  vec3 n = t.norm;
  vec3 perp = n*vec3(1.12345, 0.6543, 0.987564);
  perp *= 0.1 / perp.len();
  sep = n;

  // Find surface points using normal perturbation.
  list<vec3> cpts = collision_points(a, n, perp, t.a.a, 20);
  vec3 inv = n*-1;
  list<vec3> cpts2 = collision_points(b, inv, perp, t.a.b, 10);

  // Build list of edges from a points.
  list<edge> a_edges;
  for (list<vec3>::iterator it = cpts.begin(); it != cpts.end(); it++) {
    list<vec3>::iterator tmp = it;
    tmp++;
    if (tmp == cpts.end()) tmp = cpts.begin();
    a_edges.push_back(edge(t.a.a + *it, t.a.a + *tmp));
  }

  // Build list of edges from b points.
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

  // Find overlapping areas for surfaces.
  list<edge> edges = calculate_overlap(a_edges, b_edges, n);

  a.sim_pts.clear();
  for (list<edge>::iterator it = edges.begin(); it != edges.end(); it++) {
    // Add points to return lists.
    // @TODO Do we need both?
    a_pts.push_back(it->first);
    b_pts.push_back(it->first);

    a.sim_pts.push_back(it->first); // Set so points are highlighted for debugging
  }
  return true;
}


