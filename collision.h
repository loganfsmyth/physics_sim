
#include <vector>
#include "vec.h"


class collidable {
  public:
    virtual vec3 collision_point(vec3 dir) const = 0;
    virtual ~collidable() {}
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

bool collision_point(const collidable &a, const collidable &b, vec3 &ap, vec3 &bp, vec3 &adir, vec3 &bdir);
