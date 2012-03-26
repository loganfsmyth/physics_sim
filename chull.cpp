
#include "chull.h"
#include <limits>
#include <set>
#include <cassert>

using namespace std;

hull_edge::hull_edge(int a, int b) : a(a), b(b),refs(0) {
}

hull_face::hull_face(int e1, int e2, int e3) : e1(e1), e2(e2), e3(e3), removed(false) {
}


chull::chull(const spt &a, const spt &b, const spt &c, const spt &d) {
  add_pt(a);
  add_pt(b);
  add_pt(c);
  add_pt(d);
}
void chull::add_pt(const spt &p) {

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
    if (e.refs > 0) {
      int e1 = getEdge(e.a, pos);
      int e2 = getEdge(e.b, pos);
      addFace(*it, e1, e2);
    }
  }
}

void chull::addFace(int e1, int e2, int e3) {
  faces.push_back(hull_face(e1, e2, e3));
  edges[e1].refs += 1;
  edges[e2].refs += 1;
  edges[e3].refs += 1;
}
int chull::addEdge(int a, int b) {
  edges.push_back(hull_edge(a,b));
  return edges.size()-1;
}
int chull::getEdge(int a, int b) {
  int i = 0;
  for (elist::iterator it = edges.begin(); it != edges.end(); it++, i++) {
    if ((it->a == a && it->b == b) || (it->b == a && it->a == b)) {
      return i;
    }
  }
  addEdge(a,b);
  return edges.size()-1;
}
void chull::fRemove(hull_face &f) {
  f.removed = true;
  edges[f.e1].refs -= 1;
  edges[f.e2].refs -= 1;
  edges[f.e3].refs -= 1;
}
vec3 chull::fNorm(int f) {
  return fNorm(faces[f]);
}
vec3 chull::fNorm(hull_face &f) {
  hull_edge &ab = edges[f.e1];
  hull_edge &ac = edges[f.e2];
  vec3 norm = (pts[ab.b].val - pts[ab.a].val) * (pts[ac.b].val - pts[ac.a].val);
  if (pts[ab.a].val.dot(norm) < 0) {
    norm *= -1;
  }
  norm.norm();
  return norm;
}
pair<double,int> chull::closestFace() {
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
  return pair<double,int>(min,face);
}
epa_tri chull::getTri(int fid) {
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
