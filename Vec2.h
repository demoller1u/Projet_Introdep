#ifndef VEC2_H_
#define VEC2_H_

// ------------------------------------------------

typedef struct Vec2 {
  float x;
  float y;
} Vec2;

// ------------------------------------------------
void init_Vec( Vec2 V);

Vec2 Mult(Vec2 V, float a);

Vec2 Add(Vec2 V1, Vec2 V2);

Vec2 Sus(Vec2 V1, Vec2 V2);

float dot(Vec2 V1, Vec2 V2);

Vec2 norm(Vec2 V);


// ------------------------------------------------

#endif