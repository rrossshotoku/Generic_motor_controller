#ifndef MC_MATH_H
#define MC_MATH_H
#include <stdint.h>

/** @file mc_math.h
 *  @brief Math wrappers. Use pure C fallback or STM32G474 CORDIC/FM​​​​AC backend.
 */

void MC_Math_SinCos(float angle_rad, float *s, float *c);
float MC_Math_Sqrt(float x);
float MC_Math_Clamp(float x, float min_v, float max_v);
float MC_Math_Wrap2Pi(float angle_rad);
float MC_Math_Sign(float x);

#endif
