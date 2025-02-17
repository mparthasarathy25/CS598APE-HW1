#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__
#include "plane.h"
#include <algorithm>

class Triangle : public Plane {
public:
   Vector a, b, c;
   double thirdX;
   Triangle(Vector c, Vector b, Vector a, Texture* t);
   double getIntersection(Ray ray);
   bool getLightIntersection(Ray ray, double* fill);
   AABB calcBoundingBox() const override {
     Vector minPoint(
       std::min({a.x, b.x, c.x}),
       std::min({a.y, b.y, c.y}),
       std::min({a.z, b.z, c.z})
     );
     Vector maxPoint(
       std::max({a.x, b.x, c.x}),
       std::max({a.y, b.y, c.y}),
       std::max({a.z, b.z, c.z})
     );
     
     Vector thickness = 0.001 * vect;
     minPoint = minPoint - thickness;
     maxPoint = maxPoint + thickness;
     
     return AABB(minPoint, maxPoint);
   }
};

#endif
