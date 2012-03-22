
#include <vector>
#include <list>
#include "vec.h"


class collidable {
  public:
    virtual vec3 collision_point(vec3 dir) const = 0;
    virtual ~collidable() {}

    std::vector<vec3> sim_pts;
};

struct simplex_pt {
  vec3 val;
  vec3 a;
  vec3 b;
  simplex_pt(vec3, vec3, vec3);
  simplex_pt();
};

bool process_simplex(std::vector<simplex_pt> &pts, vec3 &dir);
bool collide(const collidable &a, const collidable &b);
bool collide(const collidable &a, const collidable &b, std::vector<simplex_pt> &pts, vec3 &dir);

vec3 collision_point(collidable &a,collidable &b, vec3 &ap, vec3 &bp, vec3 &adir, vec3 &bdir);


std::list<vec3> collision_points(collidable &a, vec3 &n, vec3 perp, vec3 &pt, int samples);

void rotateVec(vec3 &v, double angle, const vec3 &ax);

struct epa_tri {
  simplex_pt a, b, c;
  vec3 norm;
  vec3 ab, ac, bc;
  double distSq;
  epa_tri(simplex_pt &a, simplex_pt &b, simplex_pt &c);
};



epa_tri epa(collidable &one, collidable &two);
