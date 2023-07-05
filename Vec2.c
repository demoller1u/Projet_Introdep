#include "Vec2.h"
#include "math.h" 

// ------------------------------------------------

Vec2 Mult(Vec2 V, float a)
{
    Vec2 res = {a*V.x , a*V.y};
    return res;
}

Vec2 Add(Vec2 V1, Vec2 V2)
{
    Vec2 res = {V1.x + V2.x ,V1.y + V2.y };
    return res;
}

Vec2 Sus(Vec2 V1, Vec2 V2)
{
    Vec2 res = {V1.x - V2.x ,V1.y - V2.y };
    return res;
}

float dot(Vec2 V1, Vec2 V2)
{
    return V1.x * V2.x + V1.y * V2.y;
}

Vec2 norm(Vec2 V)
{
    return Mult(V,(1/sqrt(dot(V,V))));
}


// ------------------------------------------------

