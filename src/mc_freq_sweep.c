#include "mc_freq_sweep.h"
#include <math.h>

/** @file mc_freq_sweep.c
 *  @brief Stepped-sine current sweep generator (ADR-047). See mc_freq_sweep.h.
 */

#define MC_FS_TWO_PI 6.28318530717958648f

void MC_FreqSweep_Init(MC_FreqSweep_t *s)
{
    if (s == 0) { return; }
    s->start_hz = s->end_hz = s->step_hz = s->dwell_s = s->bias_a = s->amplitude_a = 0.0f;
    s->current_hz = s->phase_rad = s->dwell_elapsed_s = 0.0f;
    s->active = false;
}

void MC_FreqSweep_Start(MC_FreqSweep_t *s, float start_hz, float end_hz, float step_hz,
                        float dwell_s, float bias_a, float amplitude_a)
{
    if (s == 0) { return; }
    s->start_hz      = start_hz;
    s->end_hz        = end_hz;
    s->step_hz       = (step_hz > 0.0f) ? step_hz : 0.0f;   /* 0 => single-frequency hold */
    s->dwell_s       = (dwell_s > 0.0f) ? dwell_s : 0.0f;
    s->bias_a        = bias_a;
    s->amplitude_a   = amplitude_a;
    s->current_hz    = start_hz;
    s->phase_rad     = 0.0f;
    s->dwell_elapsed_s = 0.0f;
    s->active        = true;
}

void MC_FreqSweep_Stop(MC_FreqSweep_t *s)
{
    if (s == 0) { return; }
    s->active = false;
}

float MC_FreqSweep_Sample(MC_FreqSweep_t *s, float dt_s)
{
    if ((s == 0) || !s->active) { return 0.0f; }

    /* Advance the dwell; when it elapses, step to the next frequency (or finish). */
    s->dwell_elapsed_s += dt_s;
    if (s->dwell_elapsed_s >= s->dwell_s)
    {
        s->dwell_elapsed_s = 0.0f;
        if (s->step_hz > 0.0f)
        {
            const float next_hz = s->current_hz + s->step_hz;
            if (next_hz > s->end_hz + 1.0e-6f)         /* past the end -> done (keep current_hz in range) */
            {
                s->active = false;
                return s->bias_a;                      /* settle at the bias as it finishes */
            }
            s->current_hz = next_hz;
        }
        else
        {
            s->active = false;                         /* single-frequency hold: one dwell, then done */
            return s->bias_a;
        }
    }

    /* Continuous phase across frequency steps (no click), bounded to avoid float growth. */
    s->phase_rad += MC_FS_TWO_PI * s->current_hz * dt_s;
    while (s->phase_rad >= MC_FS_TWO_PI) { s->phase_rad -= MC_FS_TWO_PI; }
    return s->bias_a + s->amplitude_a * sinf(s->phase_rad);
}

bool MC_FreqSweep_Active(const MC_FreqSweep_t *s)
{
    return (s != 0) && s->active;
}

float MC_FreqSweep_CurrentHz(const MC_FreqSweep_t *s)
{
    return (s != 0) ? s->current_hz : 0.0f;
}
