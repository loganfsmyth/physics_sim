
#include <vector>
#include "collision.h"


struct State {
  vec3 pos;
  vec3 mo;
};
struct Derivative {
  vec3 dv;
  vec3 df;
};

struct Quaternion {
  float x, y, z, w;
};

class gameobj: public collidable {
  
  public:
  std::vector<vec3> pts;
  std::vector<int> index;
  std::vector<vec3> normals;
  int vert_per_poly;
  State st;
  State next_st;

  public:
  gameobj(vec3 c);
  virtual void calcNext(unsigned long step);
  virtual void commit();
  virtual vec3 collision_point(vec3 dir) const;
  virtual void render() const = 0;

  void rotate(double om, char axis = 'y');

  friend void triggerCollision(gameobj& a, gameobj &b, std::list<vec3> pts, vec3 &n);
};

class box : public gameobj {

  void init(vec3 c, double w, double h, double l);

  public:
  box(vec3 c, double w, double h, double l);
  box(vec3 c, double w);
  virtual void render() const;
};

class tetrahedron: public gameobj {
  void init(double w);

  public:
  tetrahedron(vec3 c, double w);
  virtual void render() const;
};


void triggerCollision(gameobj& a, gameobj &b, std::list<vec3> pts, vec3 &n);
