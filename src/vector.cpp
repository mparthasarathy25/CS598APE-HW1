#include<string.h>
#include<stdio.h>
#include<limits>
#include<math.h>
#include<stdlib.h>
//#include <printf.h>
#include <stddef.h>
#include "vector.h"
#include <immintrin.h>
#include <cmath>
#include <omp.h>


Vector::Vector(double a, double b, double c) : x(a), y(b), z(c) {
}
void Vector::operator -= (const Vector rhs) {
   x-=rhs.x; y-=rhs.y; z-=rhs.z;
}
void Vector::operator += (const Vector rhs) {
   x+=rhs.x; y+=rhs.y; z+=rhs.z;
}
void Vector::operator *= (const double rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator *= (const float rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator *= (const int rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator /= (const double rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}
void Vector::operator /= (const float rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}
void Vector::operator /= (const int rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}


Vector Vector::operator - (const Vector rhs) {
   return Vector(x-rhs.x, y-rhs.y, z-rhs.z);
}
Vector Vector::operator + (const Vector rhs) {
   return Vector(x+rhs.x, y+rhs.y, z+rhs.z);
}
/*
Vector Vector::operator * (const Vector a) {
   return Vector(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);
}*/
Vector Vector::operator * (const double rhs) {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator * (const float rhs) {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator * (const int rhs) {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator / (const double rhs) {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::operator / (const float rhs) {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::operator / (const int rhs) {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::cross(const Vector a) {
   return Vector(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);
}
double Vector::mag2(){
   return x*x+y*y+z*z; 
}
double Vector::mag(){
   return sqrt(x*x+y*y+z*z); 
}
double Vector::dot(const Vector a){
   return x*a.x+y*a.y+z*a.z;
}
Vector Vector::normalize(){
   double m = mag();
   return Vector(x/m, y/m, z/m); 
}

// Improved CPU usage from 19% to 13% and increased speedup from 1.77 seconds to 1.68 seconds
Vector solveScalers(Vector v1, Vector v2, Vector v3, Vector C) {
   // Setting variables for repetitive operations
   double d11 = v2.y * v3.z - v2.z * v3.y;
   double d12 = v2.x * v3.z - v2.z * v3.x;
   double d13 = v2.x * v3.y - v2.y * v3.x;
   double denom = v1.x * d11 - v1.y * d12 + v1.z * d13;
   // Checks whether denom is close to zero and returns a zero vector if so
   if (fabs(denom) < 1e-9) {
      return Vector(0, 0, 0);
   }
   // Setting variables for repetitive operations
   double c11 = C.y * v3.z - C.z * v3.y;
   double c12 = C.x * v3.z - C.z * v3.x;
   double c13 = C.x * v3.y - C.y * v3.x;
   double a = C.x * d11 - C.y * d12 + C.z * d13;
   double b = v1.x * c11 - v1.y * c12 + v1.z * c13;
   double c = v1.x * (v2.y * C.z - v2.z * C.y) - v1.y * (v2.x * C.z - v2.z * C.x) + v1.z * (v2.x * C.y - v2.y * C.x);
   // Division is slower than multiplication, so doing it once and then using multiplication in parallel speeds it up
   double invDenom = 1.0 / denom;
   return Vector(a * invDenom, b * invDenom, c * invDenom);
}


Ray::Ray(const Vector& po, const Vector& ve): point(po), vector(ve){}