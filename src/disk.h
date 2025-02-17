#ifndef __DISK_H__
#define __DISK_H__
#include "plane.h"
#include <cmath>
class Disk : public Plane {
public:
  Disk(const Vector &c, Texture* t, double ya, double pi, double ro, double tx, double ty);
  double getIntersection(Ray ray);
  bool getLightIntersection(Ray ray, double* fill);
  
  AABB calcBoundingBox() const override {
    return Plane::calcBoundingBox();
  }
};

#endif
