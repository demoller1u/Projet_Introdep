#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

// ------------------------------------------------

typedef struct SphereCollider {
  Vec2 center;
  float radius;
} SphereCollider;

typedef struct PlaneCollider {
  Vec2 point;
  Vec2 normale;
} PlaneCollider;

typedef struct GroundConstraints {
  Vec2 pc;
  Vec2 nc;
  int type;
} GroundConstraints;





// ------------------------------------------------

#endif