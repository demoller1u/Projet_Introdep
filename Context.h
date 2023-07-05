#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Constraint.h"

// ------------------------------------------------

typedef struct Context {
  int num_particles;
  int capacity_particles;
  Particle* particles;
  // Ground colliders 
  int num_ground_sphere;
  SphereCollider* ground_spheres;
  int num_constraints;
  GroundConstraints* ground_constraints;
} Context;

// ------------------------------------------------

Context* initializeContext(int capacity);

// ------------------------------------------------

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id);

// ------------------------------------------------

Particle getParticle(Context* context, int id);

// ------------------------------------------------

SphereCollider getGroundSphereCollider(Context* context, int id);

// ------------------------------------------------

void setDrawId(Context* context, int sphere_id, int draw_id);

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation);

// ------------------------------------------------

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider);

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider);

void checkContactWithParticle(Context* context, int particle_id_1, int particle_id_2);

// ------------------------------------------------

void applyExternalForce(Context* context, float dt);
void dampVelocities(Context* context);
void updateExpectedPosition(Context* context, float dt);
void addDynamicContactConstraints(Context* context);
void addStaticContactConstraints(Context* context);
void projectConstraints(Context* context);
void updateVelocityAndPosition(Context* context, float dt);
void applyFriction(Context* context);
void deleteContactConstraints(Context* context);

// ------------------------------------------------

