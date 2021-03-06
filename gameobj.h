
#include <vector>
#include "collision.h"
#include "quat.h"

struct State {

  // Primary
  vec3 pos;
  vec3 mo;
  Quaternion orient;
  vec3 angMo;

  // Secondary
  vec3 vel;
  Quaternion spin;
  vec3 angVel;

  // constant
  double mass;
  double inertia;

  double invMass;
  double invInertiaTensor;

  void recalc();
};

struct Derivative {
  vec3 dv;
  vec3 df;

  Quaternion spin;
  vec3 torque;
};


class gameobj: public collidable {
  
  public:
  std::vector<vec3> pts;
  std::vector<int> index;
  int vert_per_poly;
  State st;
  State next_st;
  bool picked;

  public:
  gameobj(vec3 c);
  virtual void calcNext(unsigned long step);
  virtual void commit();
  virtual vec3 collision_point(vec3 dir) const;
  virtual void render(bool pick = false) const = 0;

  void rotate(double om, char axis = 'y');

  friend void triggerCollision(gameobj& a, gameobj &b, std::list<vec3> pts, vec3 &n);
};

class box : public gameobj {

  void init(vec3 c, double w, double h, double l);

  public:
  box(vec3 c, double w, double h, double l);
  box(vec3 c, double w);
  virtual void render(bool pick = false) const;
};

class tetrahedron: public gameobj {
  void init(double w);

  public:
  tetrahedron(vec3 c, double w);
  virtual void render(bool pick = false) const;
};



class hull_obj: public gameobj {
  public:
  hull_obj(const gameobj &a, const gameobj &b);

  virtual void render(bool pick = false) const;

};


void triggerCollision(gameobj& a, gameobj &b, std::list<vec3> pts, vec3 &n);
