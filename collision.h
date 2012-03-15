
#include "vec.h"

class collidable {
  public:
    virtual vec3 collision_point(vec3 dir) const = 0;
    virtual ~collidable() {}
};

bool collide(const collidable &a, const collidable &b);
