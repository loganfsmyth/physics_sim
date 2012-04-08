
#include "quat.h"
#include <cmath>

Quaternion Quaternion::operator*(double d) const {
  Quaternion q(*this);
  q.w *= d;
  q.x *= d;
  q.y *= d;
  q.z *= d;
  return q;
}
Quaternion Quaternion::operator*(const Quaternion &q) const {
  Quaternion out;
  out.w = w*q.w - x*q.x - y*q.y - z*q.z;
  out.x = w*q.x + x*q.w + y*q.z - z*q.y;
  out.y = w*q.y - x*q.z + y*q.w + z*q.x;
  out.z = w*q.z + x*q.y - y*q.x + z*q.w;
  return out;
}
vec3 Quaternion::operator*(vec3 v) const {
  double l = v.len();
  v.norm();
  Quaternion q;
  q.x = v.x;
  q.y = v.y;
  q.z = v.z;
  q.w = 0;

  Quaternion ret = q * conj();
  ret = *this * ret;

  vec3 rv(ret.x, ret.y, ret.z);
  rv.norm();
  rv *= l;
  return rv;
}
Quaternion& Quaternion::operator+=(const Quaternion q) {
  w += q.w;
  x += q.x;
  y += q.y;
  z += q.z;
  return *this;
}
Quaternion Quaternion::operator+(const Quaternion q1) const {
  Quaternion q(*this);
  q.w += q1.w;
  q.x += q1.x;
  q.y += q1.y;
  q.z += q1.z;
  return q;
}
void Quaternion::norm() {
  double len = 1.0/sqrt(w*w + x*x + y*y + z*z);
  w *= len;
  x *= len;
  y *= len;
  z *= len;
}
Quaternion Quaternion::conj() const {
  Quaternion q(*this);
  q.x *= -1;
  q.y *= -1;
  q.z *= -1;
  return q;
}

void Quaternion::fromAxisAngle(vec3 v, double angle) {
  angle /= 2;
  v.norm();
  double s = sin(angle);
  x = v.x * s;
  y = v.y * s;
  z = v.z * s;
  w = cos(angle);
}

void Quaternion::axisAngle(vec3 &axis, double &angle) const {
  double scale = sqrt(x*x + y*y + z*z);
  if (scale == 0) {
    axis = vec3();
  }
  else {
    axis.x = x / scale;
    axis.y = y / scale;
    axis.z = z / scale;
  }
  angle = acos(w) / 2.0;
}
