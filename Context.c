#include "Context.h"
#include "Constraint.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

// ------------------------------------------------

Particle getParticle(Context* context, int id)
{
  return context->particles[id];
}

// ------------------------------------------------

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id)
{
    assert(context->num_particles<context->capacity_particles); // currently no resize in context
    context->particles[context->num_particles].position.x = x;
    context->particles[context->num_particles].position.y = y;
    context->particles[context->num_particles].velocity.x = 0.0F;
    context->particles[context->num_particles].velocity.y = 0.0F;
    context->particles[context->num_particles].inv_mass = 1.0F/mass;
    context->particles[context->num_particles].radius = radius;
    context->particles[context->num_particles].draw_id = draw_id;
    context->particles[context->num_particles].status = 0x0000;
    context->num_particles += 1;
}

// ------------------------------------------------

void setDrawId(Context* context, int sphere_id, int draw_id)
{
  context->particles[sphere_id].draw_id = draw_id;
}

// ------------------------------------------------

SphereCollider getGroundSphereCollider(Context* context, int id)
{
  return context->ground_spheres[id];
}

// ------------------------------------------------

Context* initializeContext(int capacity)
{
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));

  context->num_ground_sphere = 4;
  context->ground_spheres = malloc((context->num_ground_sphere)*sizeof(SphereCollider));
  Vec2 p0 = {-5.0f, 0.0f};
  context->ground_spheres[0].center = p0;
  context->ground_spheres[0].radius = 1.7;
  Vec2 p1 = {5.0f, 0.0f};
  context->ground_spheres[1].center = p1;
  context->ground_spheres[1].radius = 1.7;
  Vec2 p2 = {-2.0f, -2.0f};
  context->ground_spheres[2].center = p2;
  context->ground_spheres[2].radius = 1.2;
  Vec2 p3 = {2.0f, -2.0f};
  context->ground_spheres[3].center = p3;
  context->ground_spheres[3].radius = 1.2;
  
  return context;
}

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation)
{
  applyExternalForce(context, dt);
  dampVelocities(context);
  updateExpectedPosition(context, dt);
  addDynamicContactConstraints(context);
  addStaticContactConstraints(context);
 
  for(int k=0; k<num_constraint_relaxation; ++k) {
    projectConstraints(context);
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context);

  deleteContactConstraints(context);
}

// ------------------------------------------------

void applyExternalForce(Context* context, float dt)
{
  if (context->num_particles == 0) return;
  Particle *p = context->particles;
  for (int i = 0; i<context->num_particles; i++){
    (p+i)->velocity.y += - dt*(p+i)->inv_mass * 9.81;
  }
}

void dampVelocities(Context* context)
{
}

void updateExpectedPosition(Context* context, float dt)
{
  if (context->num_particles == 0) return;
  Particle *p = context->particles;
  for (int i = 0; i<context->num_particles; i++){
    (p+i)-> next_pos.x = (p+i)->position.x +dt * (p+i)->velocity.x;
    (p+i)-> next_pos.y = (p+i)->position.y +dt * (p+i)->velocity.y;
  }
}

void addDynamicContactConstraints(Context* context)
{
  for (int i = 0; i < context->num_particles; i++){ 
    for (int j = 0; j < context->num_particles; j++){
      if (j!=i){
        checkContactWithParticle(context, i, j);
      }
    }
  }
}

void addStaticContactConstraints(Context* context)
{
  for (int i = 0; i < context->num_particles; i++){
    Vec2 p = {0.0f, -8.0f};
    Vec2 n = {0.0f, 1.0f}; // on peut aussi rajouter deux plans sur les côtés pour empêcher les boules de partir du plan
    PlaneCollider plane = {p,n};
    checkContactWithPlane(context, i, &plane);
    for (int j =0; j < context->num_ground_sphere; j++){
      checkContactWithSphere(context, i, &(context->ground_spheres[j]));
    }
  }
}

void projectConstraints(Context* context)
{
}

void updateVelocityAndPosition(Context* context, float dt)
{
  if (context->num_particles==0) return;
  Particle *p = context->particles;
  for (int i = 0; i<context->num_particles; i++){
    (p+i)->velocity.x = (1.0/dt) * ((p+i)->next_pos.x - (p+i)->position.x);
    (p+i)->position.x = (p+i)->next_pos.x;
    (p+i)->velocity.y = (1.0/dt) * ((p+i)->next_pos.y - (p+i)->position.y);
    (p+i)->position.y = (p+i)->next_pos.y;
  }
}

void applyFriction(Context* context)
{
}

void deleteContactConstraints(Context* context)
{
}

// ------------------------------------------------
