#include "light.h"
#include "shape.h"
#include "camera.h"
#include <algorithm>
#include <stack>
#include <utility>
#include <vector>
      
Light::Light(const Vector & cente, unsigned char* colo) : center(cente){
   color = colo;
}

unsigned char* Light::getColor(unsigned char a, unsigned char b, unsigned char c){
   unsigned char* r = (unsigned char*)malloc(sizeof(unsigned char)*3);
   r[0] = a;
   r[1] = b;
   r[2] = c;
   return r;
}

Autonoma::Autonoma(const Camera& c): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 10;
   skybox = BLACK;
   bvhRoot = nullptr;
}

Autonoma::Autonoma(const Camera& c, Texture* tex): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 10;
   skybox = tex;
   bvhRoot = nullptr;
}

void Autonoma::addShape(Shape* r){
   ShapeNode* hi = (ShapeNode*)malloc(sizeof(ShapeNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(listStart==NULL){
      listStart = listEnd = hi;
   }
   else{
      listEnd->next = hi;
      hi->prev = listEnd;
      listEnd = hi;
   }
   // insertIntoBVH(r);
}

void Autonoma::removeShape(ShapeNode* s){
   Shape* shapeToRemove = s->data;
   
   if(s==listStart){
      if(s==listEnd){
         listStart = listEnd = NULL;
      }
      else{
         listStart = s->next;
         listStart->prev = NULL;
      }
   }
   else if(s==listEnd){
      listEnd = s->prev;
      listEnd->next = NULL;
   }
   else{
      ShapeNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   
   free(s);
}

void Autonoma::addLight(Light* r){
   LightNode* hi = (LightNode*)malloc(sizeof(LightNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(lightStart==NULL){
      lightStart = lightEnd = hi;
   }
   else{
      lightEnd->next = hi;
      hi->prev = lightEnd;
      lightEnd = hi;
   }
}

void Autonoma::removeLight(LightNode* s){
   if(s==lightStart){
      if(s==lightEnd){
         lightStart = lightStart = NULL;
      }
      else{
         lightStart = s->next;
         lightStart->prev = NULL;
      }
   }
   else if(s==lightEnd){
      lightEnd = s->prev;
      lightEnd->next = NULL;
   }
   else{
      LightNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   free(s);
}

void getLight(double* tColor, Autonoma* aut, Vector point, Vector norm, unsigned char flip) {
   tColor[0] = tColor[1] = tColor[2] = 0.;
   LightNode* t = aut->lightStart;
   
   while(t != NULL) {
      double lightColor[3];     
      lightColor[0] = t->data->color[0]/255.;
      lightColor[1] = t->data->color[1]/255.;
      lightColor[2] = t->data->color[2]/255.;
      Vector ra = t->data->center - point;
   
      ShapeNode* shapeIter = aut->listStart;
      bool hit = false;
      while(!hit && shapeIter!=NULL){
      hit = shapeIter->data->getLightIntersection(Ray(point+ra*.01, ra), lightColor);
         shapeIter = shapeIter->next;
      }
      // Ray shadowRay(point + ra * 0.01, ra);
      
      // bool hit = false;
      // std::stack<BVHNode*> nodeStack;
      // if (aut->bvhRoot) {
      //    nodeStack.push(aut->bvhRoot);
      // }
      
      // while (!hit && !nodeStack.empty()) {
      //    BVHNode* current = nodeStack.top();
      //    nodeStack.pop();
         
      //    if (!current->bounds.intersect(shadowRay)) {
      //        continue;
      //    }
         
      //    if (current->shape) {
      //        hit = current->shape->getLightIntersection(shadowRay, lightColor);
      //    } else {
      //        if (current->right) nodeStack.push(current->right);
      //        if (current->left) nodeStack.push(current->left);
      //    }
      // }

      double perc = (norm.dot(ra)/(ra.mag()*norm.mag()));
      if (!hit) {
         if (flip && perc < 0) perc = -perc;
         if (perc > 0) {
               tColor[0] += perc * lightColor[0];
               tColor[1] += perc * lightColor[1];
               tColor[2] += perc * lightColor[2];
               if (tColor[0] > 1.) tColor[0] = 1.;
               if (tColor[1] > 1.) tColor[1] = 1.;
               if (tColor[2] > 1.) tColor[2] = 1.;
         }
      }
      t = t->next;
   }
}

// bool AABB::intersect(const Ray& ray) const {
//    double t1x = (min.x - ray.point.x) * (1.0/ray.vector.x);
//    double t2x = (max.x - ray.point.x) * (1.0/ray.vector.x);
//    double t1y = (min.y - ray.point.y) * (1.0/ray.vector.y);
//    double t2y = (max.y - ray.point.y) * (1.0/ray.vector.y);
//    double t1z = (min.z - ray.point.z) * (1.0/ray.vector.z);
//    double t2z = (max.z - ray.point.z) * (1.0/ray.vector.z);
   
//    double tmin = std::max(std::max(std::min(t1x, t2x), std::min(t1y, t2y)), std::min(t1z, t2z));
//    double tmax = std::min(std::min(std::max(t1x, t2x), std::max(t1y, t2y)), std::max(t1z, t2z));
   
//    return tmax >= tmin && tmax > 0;
// }

// void AABB::expand(const AABB& other) {
//    min.x = std::min(min.x, other.min.x);
//    min.y = std::min(min.y, other.min.y);
//    min.z = std::min(min.z, other.min.z);
//    max.x = std::max(max.x, other.max.x);
//    max.y = std::max(max.y, other.max.y);
//    max.z = std::max(max.z, other.max.z);
// }

// void Autonoma::insertIntoBVH(Shape* shape) {
//    AABB newShapeBounds = shape->calcBoundingBox();
   
//    if (!bvhRoot) {
//        bvhRoot = new BVHNode();
//        bvhRoot->shape = shape;
//        bvhRoot->bounds = newShapeBounds;
//        return;
//    }

//    BVHNode* current = bvhRoot;
//    BVHNode* parent = nullptr;
//    std::vector<BVHNode*> path;
//    bool takeLeft = true;
   
//    while (!current->shape) {
//        path.push_back(current);
//        parent = current;
       
       
//        AABB leftBounds = current->left->bounds;
//        AABB rightBounds = current->right->bounds;
//        leftBounds.expand(newShapeBounds);
//        rightBounds.expand(newShapeBounds);
       
//        Vector left_vector_dims = -current->left->bounds.min + current->left->bounds.max ;
//        double leftArea = 2.0 * (left_vector_dims.x * left_vector_dims.y + left_vector_dims.y * left_vector_dims.z + left_vector_dims.z * left_vector_dims.x);
//        Vector right_vector_dims = -current->right->bounds.min + current->right->bounds.max ;
//        double rightArea = 2.0 * (right_vector_dims.x * right_vector_dims.y + right_vector_dims.y * right_vector_dims.z + right_vector_dims.z * right_vector_dims.x);
       
//        takeLeft = leftArea < rightArea;
//        current = takeLeft ? current->left : current->right;
//    }
//    BVHNode* newInternal = new BVHNode();
//    BVHNode* newLeaf = new BVHNode();
//    newLeaf->shape = shape;
//    newLeaf->bounds = newShapeBounds;
   
//    if (parent) {
//        if (takeLeft) {
//            parent->left = newInternal;
//        } else {
//            parent->right = newInternal;
//        }
//    } else {
//        bvhRoot = newInternal;
//    }
   
//    newInternal->left = newLeaf;
//    newInternal->right = current;
//    newInternal->bounds = current->bounds;
//    newInternal->bounds.expand(newShapeBounds);
   
   
//    for (auto it = path.rbegin(); it != path.rend(); ++it) {
//        (*it)->bounds = (*it)->left->bounds;
//        (*it)->bounds.expand((*it)->right->bounds);
//    }
// }
