#ifndef MC_FREQ_SWEEP_H
#define MC_FREQ_SWEEP_H
#include "mc_types.h"

/** @file mc_freq_sweep.h
 *  @brief Stepped-sine current-injection sweep for resonance / frequency-response ID (ADR-047).
 *
 *  At each frequency from @c start_hz to @c end_hz (advancing by @c step_hz) it outputs
 *  @c bias + amplitude*sin(2*pi*f*t) for @c dwell_s, then steps to the next frequency; the phase
 *  is continuous across steps (no click). It is sampled in the FAST loop (20 kHz) for fidelity --
 *  the same @ref MC_FreqSweep_Sample call advances the dwell timer + frequency step, so the whole
 *  sweep lives in one context. HAL-free / host-testable. @c step_hz = 0 holds a single frequency for
 *  one dwell. The output is a current command [A]; inject it as iq in torque mode while sweeping.
 */
typedef struct
{
    float start_hz, end_hz, step_hz, dwell_s, bias_a, amplitude_a;  /* config, latched at Start */
    float current_hz;        /* the frequency being injected right now (RO) */
    float phase_rad;         /* sinusoid phase accumulator */
    float dwell_elapsed_s;   /* time spent at the current frequency */
    bool  active;
} MC_FreqSweep_t;

void  MC_FreqSweep_Init(MC_FreqSweep_t *s);
void  MC_FreqSweep_Start(MC_FreqSweep_t *s, float start_hz, float end_hz, float step_hz,
                         float dwell_s, float bias_a, float amplitude_a);
void  MC_FreqSweep_Stop(MC_FreqSweep_t *s);

/** Advance the sweep by @p dt_s and return the current command [A] (bias + amplitude*sin).
 *  Call every fast-loop tick while active; returns 0 when not active. */
float MC_FreqSweep_Sample(MC_FreqSweep_t *s, float dt_s);

bool  MC_FreqSweep_Active(const MC_FreqSweep_t *s);
float MC_FreqSweep_CurrentHz(const MC_FreqSweep_t *s);

#endif /* MC_FREQ_SWEEP_H */
