#include "mc_notch.h"
#include <math.h>

/** @file mc_notch.c
 *  @brief RBJ band-reject biquad. See mc_notch.h. ADR-048.
 */

#define MC_NOTCH_TWO_PI 6.28318530717958648f

void MC_Notch_Init(MC_Notch_t *n)
{
    if (n == 0) { return; }
    n->b0 = 1.0f; n->b1 = 0.0f; n->b2 = 0.0f; n->a1 = 0.0f; n->a2 = 0.0f;  /* passthrough */
    n->x1 = n->x2 = n->y1 = n->y2 = 0.0f;
}

void MC_Notch_Reset(MC_Notch_t *n)
{
    if (n == 0) { return; }
    n->x1 = n->x2 = n->y1 = n->y2 = 0.0f;
}

void MC_Notch_SetParams(MC_Notch_t *n, float f0_hz, float bw_hz, float fs_hz)
{
    if (n == 0) { return; }
    /* Out-of-range -> passthrough (so a disabled/garbage config is a no-op, not an instability). */
    if ((f0_hz <= 0.0f) || (bw_hz <= 0.0f) || (fs_hz <= 0.0f) || (f0_hz >= 0.5f * fs_hz))
    {
        n->b0 = 1.0f; n->b1 = 0.0f; n->b2 = 0.0f; n->a1 = 0.0f; n->a2 = 0.0f;
        return;
    }
    const float w0    = MC_NOTCH_TWO_PI * f0_hz / fs_hz;
    const float cw    = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * (f0_hz / bw_hz));   /* Q = f0/bw */
    const float a0    = 1.0f + alpha;
    n->b0 =  1.0f / a0;
    n->b1 = -2.0f * cw / a0;
    n->b2 =  1.0f / a0;
    n->a1 = -2.0f * cw / a0;
    n->a2 = (1.0f - alpha) / a0;
}

float MC_Notch_Update(MC_Notch_t *n, float x)
{
    if (n == 0) { return x; }
    const float y = n->b0 * x + n->b1 * n->x1 + n->b2 * n->x2 - n->a1 * n->y1 - n->a2 * n->y2;
    n->x2 = n->x1; n->x1 = x;
    n->y2 = n->y1; n->y1 = y;
    return y;
}
