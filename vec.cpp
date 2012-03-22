

#include "vec.h"
#include <cmath>

std::ostream& operator<<(std::ostream& s, const vec3& v) {
  s << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return s;
}

vec3::vec3() {
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

vec3::vec3(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}


double vec3::dot(const vec3& v) const {
  return x*v.x + y*v.y + z*v.z;
}



vec3& vec3::norm() {
  if ( x || y || z) {
    double l = len();
    x /= l;
    y /= l;
    z /= l;
  }
  return *this;
}



double vec3::len() const {
  return sqrt(x*x + y*y + z*z);
}



double vec3::lenSq() const {
  return x*x + y*y + z*z;
}



vec3& vec3::negate() {
  x *= -1;
  y *= -1;
  z *= -1;
  return *this;
}


bool vec3::operator==(const vec3& v) {
  return (x == v.x && y == v.y && z == v.z);
}



bool vec3::operator!=(const vec3& v) {
  return (x != v.x || y != v.y || z != v.z);
}


vec3& vec3::operator+=(const vec3& v) {
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}



vec3& vec3::operator-=(const vec3& v) {
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}



vec3 vec3::operator*(const vec3& v) const {
  return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}



vec3 vec3::operator*(double factor) const {
  return vec3(x*factor, y*factor, z*factor);
}



vec3& vec3::operator*=(double factor) {
  x *= factor;
  y *= factor;
  z *= factor;
  return *this;
}

vec3 vec3::operator-(const vec3& v) const {
  return vec3(x-v.x, y-v.y, z-v.z);
}
vec3 vec3::operator+(const vec3& v) const {
  return vec3(x+v.x, y+v.y, z+v.z);
}
