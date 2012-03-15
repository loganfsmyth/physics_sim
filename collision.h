

class collidable {
  public:
    virtual vec3 collide_point(vec3 dir) = 0;
    virtual ~collidable() {}
}
