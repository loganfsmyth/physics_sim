
#include <iostream>
#include "collision.h"

using namespace std;

class sphere: public collidable {
  double x, y, z, r;
  public:
  sphere(double nx, double ny, double nz, double nr) {
    x = nx;
    y = ny;
    z = nz;
    r = nr;
  }
  vec3 collision_point(vec3 dir) const{
    double l = dir.len();
    if (l == 0) {
      dir = vec3(0.0, 0.0, 0.0);
    }
    else {
      dir *= r/l;
    }
    return vec3(x, y, z) + dir;
  }
};

int main(int argc, char** argv) {


  sphere one(-1, 0, 0, 1);
  sphere two(1, 0, 0, 1.0);

  cout << (collide(one, two) ? "HIT" : "NO HIT") << endl;

}
