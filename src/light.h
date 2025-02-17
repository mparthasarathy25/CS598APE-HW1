#ifndef __LIGHT_H__
#define __LIGHT_H__
#include "vector.h"
#include "camera.h"
#include "Textures/texture.h"
#include "Textures/colortexture.h"
#include <vector>

class Light{
  public:
   unsigned char* color;
   unsigned char* getColor(unsigned char a, unsigned char b, unsigned char c);
   Vector center;
   Light(const Vector & cente, unsigned char* colo);
};

struct LightNode{
   Light* data;
   LightNode* prev, *next;
};

class Shape;
struct ShapeNode{
   Shape* data;
   ShapeNode* prev, *next;
};

class AABB {
public:
    Vector min, max;
    AABB() : min(Vector(INFINITY, INFINITY, INFINITY)), max(Vector(-INFINITY, -INFINITY, -INFINITY)) {}
    AABB(const Vector& min, const Vector& max) : min(min), max(max) {}
    bool intersect(const Ray& ray) const;
    void expand(const AABB& other);
};

struct BVHNode {
    AABB bounds;
    BVHNode* left;
    BVHNode* right;
    Shape* shape;
    
    BVHNode() : left(nullptr), right(nullptr), shape(nullptr) {}
};


class Autonoma{
public:
   Camera camera;
   Texture* skybox;
   unsigned int depth;
   ShapeNode *listStart, *listEnd;
   LightNode *lightStart, *lightEnd;
   BVHNode* bvhRoot;
   Autonoma(const Camera &c);
   Autonoma(const Camera &c, Texture* tex);
   void addShape(Shape* s);
   void removeShape(ShapeNode* s);
   void addLight(Light* s);
   void removeLight(LightNode* s);
   void insertIntoBVH(Shape* shape);
};

void getLight(double* toFill, Autonoma* aut, Vector point, Vector norm, unsigned char r);

#endif
