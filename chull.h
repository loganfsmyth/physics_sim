
#ifndef INCLUDE_CHULL_H
#define INCLUDE_CHULL_H

#include <vector>
#include <boost/tuple/tuple.hpp>
#include "collision.h"

struct hull_edge {
  int a, b;
  int refs;
  hull_edge(int a, int b);
};

struct hull_face {
  int e1, e2, e3;
  bool removed;
  hull_face(int e1, int e2, int e3);
};


class chull {
  typedef simplex_pt spt;
  typedef std::vector<simplex_pt> plist;
  typedef std::vector<hull_edge> elist;
  typedef std::vector<hull_face> flist;

  plist pts;
  elist edges;
  flist faces;

  public:
  chull();
  chull(const spt &a, const spt &b, const spt &c, const spt &d);
  void add_pt(const spt &p);
  void addFace(int e1, int e2, int e3);
  int addEdge(int a, int b);
  int getEdge(int a, int b);
  void fRemove(hull_face &f);
  vec3 fNorm(int f);
  vec3 fNorm(hull_face &f);
  std::pair<double,int> closestFace();
  epa_tri getTri(int fid);
  epa_tri getTri(const hull_face &f);

  std::vector<boost::tuple<vec3,vec3,vec3> > getFaces();
};


#endif
