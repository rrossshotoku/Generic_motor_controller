#include "mc_math.h"
#include <math.h>

/** @file mc_math.c
 *  @brief Pure-C math wrappers (libm fallback). CORDIC/FMAC backend may be added later.
 */

#define MC_MATH_TWO_PI 6.28318530717958647692f

void MC_Math_SinCos(float angle_rad, float *s, float *c)
{
    *s = sinf(angle_rad);
    *c = cosf(angle_rad);
}

float MC_Math_Sqrt(float x)
{
    return sqrtf(x);
}

float MC_Math_Clamp(float x, float min_v, float max_v)
{
    if (x < min_v) { return min_v; }
    if (x > max_v) { return max_v; }
    return x;
}

float MC_Math_Wrap2Pi(float angle_rad)
{
    while (angle_rad >= MC_MATH_TWO_PI) { angle_rad -= MC_MATH_TWO_PI; }
    while (angle_rad < 0.0f)            { angle_rad += MC_MATH_TWO_PI; }
    return angle_rad;
}

float MC_Math_Sign(float x)
{
    if (x > 0.0f) { return 1.0f; }
    if (x < 0.0f) { return -1.0f; }
    return 0.0f;
}
