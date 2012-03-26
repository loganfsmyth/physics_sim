
#ifndef INCLUDE_QUAT_H
#define INCLUDE_QUAT_H

#include "vec.h"

struct Quaternion {
  float x, y, z, w;
  Quaternion operator*(double d) const;
  Quaternion operator*(const Quaternion &q) const;
  Quaternion& operator+=(const Quaternion q);
  Quaternion operator+(const Quaternion q1) const;
  void axisAngle(vec3 &axis, double &angle) const;
  void norm();
  void fromAxisAngle(vec3 v, double angle);
  Quaternion conj() const;
  vec3 operator*(vec3 v) const;
};


#endif
