#include "shape.h"
#include <vector>
#include <cstdlib>
#include <algorithm>

Shape::Shape(const Vector &c, Texture* t, double ya, double pi, double ro): center(c), texture(t), yaw(ya), pitch(pi), roll(ro){
};

inline void Shape::setAngles(double a, double b, double c){
   yaw =a; pitch = b; roll = c;
   xcos = cos(yaw);
   xsin = sin(yaw);
   ycos = cos(pitch);
   ysin = sin(pitch);
   zcos = cos(roll);
   zsin = sin(roll);
}

inline void Shape::setYaw(double a){
   yaw =a;
   xcos = cos(yaw);
   xsin = sin(yaw);
}

inline void Shape::setPitch(double b){
   pitch = b;
   ycos = cos(pitch);
   ysin = sin(pitch);
}

inline void Shape::setRoll(double c){
   roll = c;
   zcos = cos(roll);
   zsin = sin(roll);
}

typedef struct {
   double time;
   Shape* shape;
} TimeAndShape;

// This is the third/final iteration of changes we made where we manually added a variable called listSize to the Autonoma constructor so that it does not need to be counted everytime, but can be retrieved from the constructor directly
// This reduced runtime to 1.77 seconds and reduced CPU occupation to about 4.04%
void calcColor(unsigned char* toFill,Autonoma* c, Ray ray, unsigned int depth){
   ShapeNode* t = c->listStart;
   size_t seen = 0;
   TimeAndShape min;
   //removed malloc for times and performed single pass to retrieve minimum TimeAndShape from c list
   //time went from 1.68 to 1.58
   while (t != NULL) {
      double time = t->data->getIntersection(ray);
      if (seen == 0) {
         min = (TimeAndShape){ time, t->data };
      } else {
         TimeAndShape tmp = (TimeAndShape){ time, t->data };
         if (tmp.time < min.time) {
            min = tmp;
         }
      }
      seen++;
      t = t->next;
   }

   if (seen == 0 || min.time == inf) {
      double opacity, reflection, ambient;
      Vector temp = ray.vector.normalize();
      const double x = temp.x;
      const double z = temp.z;
      const double me = (temp.y<0)?-temp.y:temp.y;
      const double angle = atan2(z, x);
      c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
      return;
   }

   double curTime = min.time;
   Shape* curShape = min.shape;

   Vector intersect = curTime*ray.vector+ray.point;
   double opacity, reflection, ambient;
   curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
   double lightData[3];
   getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
   toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
   toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
   toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
   if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
      unsigned char col[4];
      if(opacity<1-1e-6){
         Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
         calcColor(col, c, nextRay, depth+1);
         toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
         toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
         toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
      }
      if(reflection>1e-6){
         Vector norm = curShape->getNormal(intersect).normalize();
         Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
         Ray nextRay = Ray(intersect+vec*1E-4, vec);
         calcColor(col, c, nextRay, depth+1);
      
         toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
         toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
         toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
      }
   }
}

// This is the third iteration of changes where we used a stack to traverse the BVH and find the closest intersection
// This led to a speedup to about 0.41 seconds
/*
void calcColor(unsigned char* toFill, Autonoma* c, Ray ray, unsigned int depth) {
    TimeAndShape min = {inf, nullptr};
    
    if (c->bvhRoot) {
        std::vector<BVHNode*> stack;
        stack.push_back(c->bvhRoot);
        
        while (!stack.empty()) {
            BVHNode* node = stack.back();
            stack.pop_back();
            
            if (!node->bounds.intersect(ray)) continue;
            
            if (node->shape) {
                double time = node->shape->getIntersection(ray);
                if (time < min.time && time > 0) {  // Added time > 0 check
                    min = {time, node->shape};
                }
            } else {
                if (node->left) stack.push_back(node->left);
                if (node->right) stack.push_back(node->right);
            }
        }
    }
    
    if (min.time == inf || !min.shape) {
        double opacity, reflection, ambient;
        Vector temp = ray.vector.normalize();
        const double x = temp.x;
        const double z = temp.z;
        const double me = (temp.y<0)?-temp.y:temp.y;
        const double angle = atan2(z, x);
        c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
        return;
    }
    
    double curTime = min.time;
    Shape* curShape = min.shape;

    Vector intersect = curTime*ray.vector+ray.point;
    double opacity, reflection, ambient;
    curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
    double lightData[3];
    getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
    toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
    toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
    toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
    if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
        unsigned char col[4];
        if(opacity<1-1e-6){
            Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
            calcColor(col, c, nextRay, depth+1);
            toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
            toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
            toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
        }
        if(reflection>1e-6){
            Vector norm = curShape->getNormal(intersect).normalize();
            Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
            Ray nextRay = Ray(intersect+vec*1E-4, vec);
            calcColor(col, c, nextRay, depth+1);
          
            toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
            toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
            toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
        }
    }
}
*/

// This is the second iteration of changes we did to the code where we changed the capacity of the array so that when we allocated and reallocated the memory, it could hold more
// This increased speedup to about 1.78 seconds and reduced the CPU occupation to about 4.5%

// void calcColor(unsigned char* toFill,Autonoma* c, Ray ray, unsigned int depth){
//    ShapeNode* t = c->listStart;
//    size_t capacity = 16;
//    size_t seen = 0;
//    TimeAndShape* times = (TimeAndShape*)malloc(capacity * sizeof(TimeAndShape));
//    while (t != NULL) {
//       double time = t->data->getIntersection(ray);
//       if (seen == capacity) {
//          capacity *= 2;
//          TimeAndShape *times2 = (TimeAndShape*)realloc(times, capacity * sizeof(TimeAndShape));
//          if (!times2) {
//             free(times);
//             return;
//          }
//          times = times2;
//       }
//       times[seen] = (TimeAndShape){ time, t->data };
//       seen ++;
//       t = t->next;
//    }
//    insertionSort(times, seen);
//    if (seen == 0 || times[0].time == inf) {
//       double opacity, reflection, ambient;
//       Vector temp = ray.vector.normalize();
//       const double x = temp.x;
//       const double z = temp.z;
//       const double me = (temp.y<0)?-temp.y:temp.y;
//       const double angle = atan2(z, x);
//       c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
//       return;
//    }

//    double curTime = times[0].time;
//    Shape* curShape = times[0].shape;
//    free(times);

//    Vector intersect = curTime*ray.vector+ray.point;
//    double opacity, reflection, ambient;
//    curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
//    double lightData[3];
//    getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
//    toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
//    toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
//    toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
//    if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
//       unsigned char col[4];
//       if(opacity<1-1e-6){
//          Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
//          calcColor(col, c, nextRay, depth+1);
//          toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
//          toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
//          toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
//       }
//       if(reflection>1e-6){
//          Vector norm = curShape->getNormal(intersect).normalize();
//          Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
//          Ray nextRay = Ray(intersect+vec*1E-4, vec);
//          calcColor(col, c, nextRay, depth+1);
      
//          toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
//          toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
//          toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
//       }
//    }
// }


// This is the first iteration of changes where we manually counted the number of elements in the array and then allocated memory accordingly
// This led to a speedup to 1.8 seconds but 5.5% CPU occupation

// void calcColor(unsigned char* toFill,Autonoma* c, Ray ray, unsigned int depth){
//    ShapeNode* t = c->listStart;
//    size_t seen = 0;
//    ShapeNode* counter = c->listStart;
//    while (counter) {
//       seen++;
//       counter = counter->next;
//    }
//    TimeAndShape* times = (TimeAndShape*)malloc(sizeof(TimeAndShape) * seen);
//    seen = 0;
//    t = c->listStart;
//    while (t != NULL) {
//       times[seen++] = { t->data->getIntersection(ray), t->data };
//       t = t->next;
//     }
//    insertionSort(times, seen);
//    if (seen == 0 || times[0].time == inf) {
//       free(times);
//       double opacity, reflection, ambient;
//       Vector temp = ray.vector.normalize();
//       const double x = temp.x;
//       const double z = temp.z;
//       const double me = (temp.y<0)?-temp.y:temp.y;
//       const double angle = atan2(z, x);
//       c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
//       return;
//    }

//    double curTime = times[0].time;
//    Shape* curShape = times[0].shape;
//    free(times);

//    Vector intersect = curTime*ray.vector+ray.point;
//    double opacity, reflection, ambient;
//    curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
//    double lightData[3];
//    getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
//    toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
//    toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
//    toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
//    if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
//       unsigned char col[4];
//       if(opacity<1-1e-6){
//          Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
//          calcColor(col, c, nextRay, depth+1);
//          toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
//          toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
//          toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
//       }
//       if(reflection>1e-6){
//          Vector norm = curShape->getNormal(intersect).normalize();
//          Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
//          Ray nextRay = Ray(intersect+vec*1E-4, vec);
//          calcColor(col, c, nextRay, depth+1);
      
//          toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
//          toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
//          toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
//       }
//    }
// }


// This is the Original code that had 17% CPU and took 2.25 seconds to run

// void calcColor(unsigned char* toFill,Autonoma* c, Ray ray, unsigned int depth){
//    ShapeNode* t = c->listStart;
//    TimeAndShape *times = (TimeAndShape*)malloc(0);
//    size_t seen = 0;
//    while(t!=NULL){
//       double time = t->data->getIntersection(ray);

//       TimeAndShape *times2 = (TimeAndShape*)malloc(sizeof(TimeAndShape)*(seen + 1));
//       for (int i=0; i<seen; i++)
//          times2[i] = times[i];
//       times2[seen] = (TimeAndShape){ time, t->data };
//       free(times);
//       times = times2;
//       seen ++;
//       t = t->next;
//    }
//    insertionSort(times, seen);
//    if (seen == 0 || times[0].time == inf) {
//       double opacity, reflection, ambient;
//       Vector temp = ray.vector.normalize();
//       const double x = temp.x;
//       const double z = temp.z;
//       const double me = (temp.y<0)?-temp.y:temp.y;
//       const double angle = atan2(z, x);
//       c->skybox->getColor(toFill, &ambient, &opacity, &reflection, fix(angle/M_TWO_PI),fix(me));
//       return;
//    }

//    double curTime = times[0].time;
//    Shape* curShape = times[0].shape;
//    free(times);

//    Vector intersect = curTime*ray.vector+ray.point;
//    double opacity, reflection, ambient;
//    curShape->getColor(toFill, &ambient, &opacity, &reflection, c, Ray(intersect, ray.vector), depth);
   
//    double lightData[3];
//    getLight(lightData, c, intersect, curShape->getNormal(intersect), curShape->reversible());
//    toFill[0] = (unsigned char)(toFill[0]*(ambient+lightData[0]*(1-ambient)));
//    toFill[1] = (unsigned char)(toFill[1]*(ambient+lightData[1]*(1-ambient)));
//    toFill[2] = (unsigned char)(toFill[2]*(ambient+lightData[2]*(1-ambient)));
//    if(depth<c->depth && (opacity<1-1e-6 || reflection>1e-6)){
//       unsigned char col[4];
//       if(opacity<1-1e-6){
//          Ray nextRay = Ray(intersect+ray.vector*1E-4, ray.vector);
//          calcColor(col, c, nextRay, depth+1);
//          toFill[0]= (unsigned char)(toFill[0]*opacity+col[0]*(1-opacity));
//          toFill[1]= (unsigned char)(toFill[1]*opacity+col[1]*(1-opacity));
//          toFill[2]= (unsigned char)(toFill[2]*opacity+col[2]*(1-opacity));        
//       }
//       if(reflection>1e-6){
//          Vector norm = curShape->getNormal(intersect).normalize();
//          Vector vec = ray.vector-2*norm*(norm.dot(ray.vector));
//          Ray nextRay = Ray(intersect+vec*1E-4, vec);
//          calcColor(col, c, nextRay, depth+1);
      
//          toFill[0]= (unsigned char)(toFill[0]*(1-reflection)+col[0]*(reflection));
//          toFill[1]= (unsigned char)(toFill[1]*(1-reflection)+col[1]*(reflection));
//          toFill[2]= (unsigned char)(toFill[2]*(1-reflection)+col[2]*(reflection));
//       }
//    }
// }

