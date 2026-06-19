#include "mc_math.h"
#include <math.h>
void MC_Math_SinCos(float angle_rad, float *s, float *c) { if (s) *s = sinf(angle_rad); if (c) *c = cosf(angle_rad); }
float MC_Math_Sqrt(float x) { return sqrtf(x); }
float MC_Math_Clamp(float x, float min_v, float max_v) { if (x < min_v) return min_v; if (x > max_v) return max_v; return x; }
float MC_Math_Wrap2Pi(float a) { const float two_pi = 6.283185307179586f; while (a >= two_pi) a -= two_pi; while (a < 0.0f) a += two_pi; return a; }
float MC_Math_Sign(float x) { return (x > 0.0f) - (x < 0.0f); }
