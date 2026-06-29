#ifndef MC_NOTCH_H
#define MC_NOTCH_H
#include "mc_types.h"

/** @file mc_notch.h
 *  @brief 2nd-order IIR band-reject (notch) biquad — RBJ cookbook. Attenuates a band centred on
 *  @c f0 with -3 dB edges at @c f0 ± bw/2 (deep null at f0). @ref MC_Notch_SetParams recomputes the
 *  coefficients (uses trig — call it off the fast loop, e.g. when the params change); @ref
 *  MC_Notch_Update is a fast-loop-safe biquad (no trig). HAL-free / host-testable. See ADR-048.
 */
typedef struct
{
    float b0, b1, b2, a1, a2;   /* normalised coefficients (a0 divided out) */
    float x1, x2, y1, y2;       /* input/output history */
} MC_Notch_t;

void  MC_Notch_Init(MC_Notch_t *n);                 /* passthrough + clear state */
void  MC_Notch_Reset(MC_Notch_t *n);                /* clear state, keep coefficients */
void  MC_Notch_SetParams(MC_Notch_t *n, float f0_hz, float bw_hz, float fs_hz);  /* invalid -> passthrough */
float MC_Notch_Update(MC_Notch_t *n, float x);      /* one sample */

#endif /* MC_NOTCH_H */
