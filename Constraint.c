#include "Context.h"
#include "Constraint.h"
#include "Particle.h"
#include "math.h"

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider)
{
    Vec2 difference = Sus(context->particles[particle_id].next_pos, collider->point);
    float critera = dot(difference,collider->normale) - context->particles[particle_id].radius;
    if (critera < 0.0F){
        float a = dot(Sus(context->particles[particle_id].next_pos,collider->point),collider->normale);
        Vec2 qc = Sus(context->particles[particle_id].next_pos, Mult(collider->normale,a));
        float C = dot(Sus(context->particles[particle_id].next_pos,qc),collider->normale) - context->particles[particle_id].radius ; // rebond position ici limite
        Vec2 delta = Mult(collider->normale, -C);
        context->particles[particle_id].next_pos = Add(context->particles[particle_id].next_pos,delta);
    } 
}


void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider)
{
    Vec2 difference = Sus(context->particles[particle_id].next_pos, collider->center);
    float sdf = sqrt(dot(difference,difference)) - (collider->radius + context->particles[particle_id].radius); // rajoute rayon particule
    if (sdf < 0.0f){
        Vec2 nc = norm(Sus(context->particles[particle_id].next_pos,collider->center)); // rebond position ici limite
        Vec2 pc = Sus(context->particles[particle_id].next_pos, Mult(nc,sdf));
        PlaneCollider plane ={pc,nc};
        float a = dot(Sus(context->particles[particle_id].next_pos,pc),nc);
        Vec2 qc = Sus(context->particles[particle_id].next_pos, Mult(nc,a));
        float C = dot(Sus(context->particles[particle_id].next_pos,qc),nc); // enlève le rayon
        Vec2 delta = Mult(nc, -C);
        context->particles[particle_id].next_pos = Add(context->particles[particle_id].next_pos,delta);
    } 
}


void checkContactWithParticle(Context* context, int particle_id_1, int particle_id_2) // 1 est celle rentrant en contact avec 2 (regarde 1 en priorité)
{
    Vec2 difference = Sus(context->particles[particle_id_1].next_pos, context->particles[particle_id_2].next_pos);
    float sdf = sqrt(dot(difference,difference)) - (context->particles[particle_id_2].radius + context->particles[particle_id_1].radius); // rajoute rayon particule
    if (sdf < 0.0f){
        Vec2 nc = norm(Sus(context->particles[particle_id_1].next_pos, context->particles[particle_id_2].next_pos)); // rebond position ici limite
        Vec2 pc = Sus(context->particles[particle_id_1].next_pos, Mult(nc,sdf));
        PlaneCollider plane ={pc,nc};
        float a = dot(Sus(context->particles[particle_id_1].next_pos,pc),nc);
        Vec2 qc = Sus(context->particles[particle_id_1].next_pos, Mult(nc,a));
        float C = dot(Sus(context->particles[particle_id_1].next_pos,qc),nc); // enlève le rayon
        Vec2 delta = Mult(nc, -C);
        context->particles[particle_id_1].next_pos = Add(context->particles[particle_id_1].next_pos,delta);
    } 
}




