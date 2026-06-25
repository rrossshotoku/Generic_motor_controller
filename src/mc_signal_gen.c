#include "mc_signal_gen.h"
#include <math.h>

/** @file mc_signal_gen.c
 *  @brief Reference test-signal generator (ADR-030). State flow:
 *         IDLE -> RAMP(to peak) -> DWELL(peak, dwell_s) -> RAMP(to 0) ->
 *             continuous ? DWELL(0, pause_s = inter-pulse pause) -> flip -> repeat : IDLE.
 *         rate <= 0 makes a RAMP an instantaneous step. The peak dwell (dwell_s) and the inter-pulse
 *         pause (pause_s) are independent (0x2910:4 and 0x2910:9). max_accel > 0 makes each RAMP
 *         acceleration-limited (trapezoidal velocity, cruise rate); 0 = constant-rate linear ramp (ADR-032).
 */

void MC_SignalGen_Init(MC_SignalGen_t *g)
{
    if (g == 0) { return; }
    g->state = MC_SIG_IDLE;
    g->peak = 0.0f; g->rate = 0.0f; g->dwell_s = 0.0f; g->pause_s = 0.0f; g->max_accel = 0.0f; g->continuous = false;
    g->target = 0.0f; g->to_peak = false; g->value = 0.0f; g->vel = 0.0f; g->dwell_elapsed_s = 0.0f;
}

void MC_SignalGen_Start(MC_SignalGen_t *g, float amplitude, float rate, float dwell_s, float pause_s,
                        float max_accel, bool continuous)
{
    if (g == 0) { return; }
    g->peak            = amplitude;
    g->rate            = (rate      > 0.0f) ? rate      : 0.0f;   /* <= 0 => step */
    g->dwell_s         = (dwell_s   > 0.0f) ? dwell_s   : 0.0f;
    g->pause_s         = (pause_s   > 0.0f) ? pause_s   : 0.0f;   /* inter-pulse pause (continuous mode) */
    g->max_accel       = (max_accel > 0.0f) ? max_accel : 0.0f;  /* >0 => trapezoidal (accel-limited) ramps */
    g->continuous      = continuous;
    g->target          = amplitude;
    g->to_peak         = true;
    g->value           = 0.0f;
    g->vel             = 0.0f;
    g->dwell_elapsed_s = 0.0f;
    g->state           = MC_SIG_RAMP;
}

void MC_SignalGen_Stop(MC_SignalGen_t *g)
{
    if (g == 0) { return; }
    /* Stop repeating and ramp the output back to 0, then idle (bumpless). */
    g->continuous = false;
    g->target     = 0.0f;
    g->to_peak    = false;
    g->state      = (g->value == 0.0f) ? MC_SIG_IDLE : MC_SIG_RAMP;
}

bool MC_SignalGen_Active(const MC_SignalGen_t *g)
{
    return (g != 0) && (g->state != MC_SIG_IDLE);
}

float MC_SignalGen_Velocity(const MC_SignalGen_t *g)
{
    return (g != 0) ? g->vel : 0.0f;
}

/* Move @p v toward @p target by @p step (>= 0). Returns true once it reaches/passes the target. */
static bool approach(float *v, float target, float step)
{
    if (*v < target) { *v += step; if (*v >= target) { *v = target; return true; } }
    else             { *v -= step; if (*v <= target) { *v = target; return true; } }
    return false;
}

/* Clamp @p x to [lo, hi]. */
static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/* Acceleration-limited step toward g->target: trapezoidal velocity (cruise g->rate, accel g->max_accel).
 * The (signed) velocity is rate-limited to max_accel*dt per tick, so the FF acceleration is bounded by
 * max_accel *everywhere* (including the landing); the desired speed is capped to the braking speed
 * sqrt(2*a*dist) so it can always decelerate to rest at the target (triangular if the move is too short to
 * reach cruise). Updates g->value and g->vel (the FF velocity); returns true once the target is reached. */
static bool trapezoid_step(MC_SignalGen_t *g, float dt)
{
    const float to_go   = g->target - g->value;
    const float dist    = fabsf(to_go);
    const float dir     = (to_go >= 0.0f) ? 1.0f : -1.0f;
    const float dv      = g->max_accel * dt;
    const float v_brake = sqrtf(2.0f * g->max_accel * dist);            /* max speed that can still stop */
    const float cruise  = (g->rate < v_brake) ? g->rate : v_brake;
    g->vel    = clampf(dir * cruise, g->vel - dv, g->vel + dv);         /* rate-limited -> |accel| <= max */
    g->value += g->vel * dt;
    if (((g->target - g->value) * dir) <= 0.0f)        /* reached/passed the target */
    {
        g->value = g->target;                           /* hold here while the FF velocity winds down to 0 */
        if (fabsf(g->vel) < 1.0e-9f) { return true; }   /* clampf has driven it to exactly 0 -> settled */
    }
    return false;
}

float MC_SignalGen_Update(MC_SignalGen_t *g, float dt_s)
{
    if (g == 0) { return 0.0f; }

    switch (g->state)
    {
    case MC_SIG_RAMP:
    {
        bool reached;
        if (g->rate <= 0.0f)
        {
            g->value = g->target; g->vel = 0.0f; reached = true;                /* step edge: no FF velocity */
        }
        else if (g->max_accel > 0.0f)
        {
            reached = trapezoid_step(g, dt_s);                                  /* accel-limited (ADR-032) */
        }
        else
        {
            g->vel  = (g->target > g->value) ? g->rate : -g->rate;             /* linear ramp velocity (the FF) */
            reached = approach(&g->value, g->target, g->rate * dt_s);
        }
        if (reached)
        {
            if (g->to_peak)
            {
                g->state           = MC_SIG_DWELL;
                g->dwell_elapsed_s = 0.0f;
            }
            else if (g->continuous)        /* back at 0 -> pause, then flip for the next pulse */
            {
                g->state           = MC_SIG_DWELL;   /* dwell at 0 = the inter-pulse pause */
                g->dwell_elapsed_s = 0.0f;
            }
            else
            {
                g->state = MC_SIG_IDLE;    /* back at 0, one-shot -> done */
            }
        }
        break;
    }
    case MC_SIG_DWELL:
    {
        g->value            = g->target;
        g->vel              = 0.0f;
        g->dwell_elapsed_s += dt_s;
        const float hold = (g->target != 0.0f) ? g->dwell_s : g->pause_s;   /* peak dwell vs inter-pulse pause */
        if (g->dwell_elapsed_s >= hold)
        {
            if (g->target != 0.0f)         /* dwelt at a peak -> ramp back to 0 */
            {
                g->target  = 0.0f;
                g->to_peak = false;
            }
            else                            /* paused at 0 (continuous) -> flip + start the next pulse */
            {
                g->peak    = -g->peak;
                g->target  =  g->peak;
                g->to_peak =  true;
            }
            g->state = MC_SIG_RAMP;
        }
        break;
    }

    case MC_SIG_IDLE:
    default:
        g->value = 0.0f;
        g->vel   = 0.0f;
        break;
    }
    return g->value;
}
