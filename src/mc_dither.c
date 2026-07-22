/*
 * mc_dither -- low-speed anti-stiction current dither (ADR-066).
 * Zero-mean sine, faded by speed. HAL-free; statics initialise to the disabled state.
 */

#include "mc_dither.h"

#include <math.h>

#define MC_DITHER_TWO_PI  (6.28318530718f)

static bool  s_enable = false;
static float s_thresh = 0.0f;   /* speed threshold [rad/s] */
static float s_amp    = 0.0f;   /* amplitude [A] */
static float s_freq   = 0.0f;   /* frequency [Hz] */
static float s_phase  = 0.0f;   /* oscillator phase [0, 2pi) */

void MC_Dither_Init(void)
{
    s_enable = false;
    s_thresh = 0.0f;
    s_amp    = 0.0f;
    s_freq   = 0.0f;
    s_phase  = 0.0f;
}

void MC_Dither_SetParams(bool enable, float speed_threshold_rad_s, float amplitude_a, float freq_hz)
{
    s_enable = enable;
    s_thresh = speed_threshold_rad_s;
    s_amp    = amplitude_a;
    s_freq   = freq_hz;
}

float MC_Dither_Update(float velocity_rad_per_s, float dt_s)
{
    if (!s_enable || (s_amp <= 0.0f) || (s_freq <= 0.0f) || (s_thresh <= 0.0f) || (dt_s <= 0.0f))
    {
        s_phase = 0.0f;
        return 0.0f;
    }

    /* Speed fade: full amplitude at v=0, linear to 0 at the threshold, off above it. */
    const float av   = fabsf(velocity_rad_per_s);
    float       fade = (s_thresh - av) / s_thresh;
    if (fade <= 0.0f)
    {
        s_phase = 0.0f;          /* above threshold: off, reset phase for a clean re-entry */
        return 0.0f;
    }
    if (fade > 1.0f) { fade = 1.0f; }

    /* Advance the phase accumulator and emit a zero-mean sine. */
    s_phase += MC_DITHER_TWO_PI * s_freq * dt_s;
    if (s_phase >= MC_DITHER_TWO_PI) { s_phase -= MC_DITHER_TWO_PI; }

    return s_amp * fade * sinf(s_phase);
}
