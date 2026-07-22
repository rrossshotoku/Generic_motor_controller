#include "mc_homing.h"
#include "mc_if_od.h"   /* MC_IF_HOME_* status codes (frozen contract, HAL-free) */
#include <math.h>

/** @file mc_homing.c
 *  @brief Home-to-hard-stop sequencer — see mc_homing.h and ADR-057 / ADR-068.
 *
 *  Behaviour is a 1:1 extraction of the former inlined logic in mc_scheduler.c: an approach at
 *  home_velocity toward a hard stop, the stop detected by a no-movement dwell (armed only after the
 *  axis has moved, so the initial ramp-up isn't mistaken for the stop) OR an over-current trip; the
 *  datum is captured at the stop; then a fixed-time back-off parks the axis clear before finishing.
 *  A safety timeout fails the routine if neither trips. Timing is in medium-loop (1 kHz) ticks.
 */

#define MC_HOME_STILL_MS   1000u  /* stop confirmed after movement stays negligible this long [ms @ 1 kHz] */
#define MC_HOME_BACKOFF_MS 1000u  /* after the stop, drive OPPOSITE this long, then finish [ms @ 1 kHz] */
#define MC_HOME_STILL_EPS  0.01f  /* |mechanical velocity| below this counts as "not moving" [rad/s] */
#define MC_HOME_TIMEOUT_MS 30000u /* safety abort if it never settles / trips [ms @ 1 kHz] */

void MC_Homing_Init(MC_Homing_t *h)
{
    h->status        = MC_IF_HOME_IDLE;
    h->backing_off   = false;
    h->moved         = false;
    h->still_ticks   = 0u;
    h->total_ticks   = 0u;
    h->backoff_ticks = 0u;
}

void MC_Homing_Update(MC_Homing_t *h, const MC_HomingInput_t *in, MC_HomingOutput_t *out)
{
    out->active          = false;
    out->want_drive      = false;
    out->velocity_cmd    = 0.0f;
    out->slew            = false;
    out->reset_slew      = false;
    out->capture_zero    = false;
    out->consume_oc_trip = false;
    out->completed       = false;

    /* Start / abort / clear (home_command is a level; 0 also clears a latched DONE/FAILED). */
    if (in->enable)
    {
        if (h->status == MC_IF_HOME_IDLE)   /* rising into an idle state -> start */
        {
            h->status        = MC_IF_HOME_RUNNING;
            h->still_ticks   = 0u;
            h->total_ticks   = 0u;
            h->moved         = false;
            h->backing_off   = false;
            h->backoff_ticks = 0u;
            out->consume_oc_trip = true;   /* drop a stale latched trip so it can't instantly "find" the stop */
            out->reset_slew      = true;   /* ramp the approach from the current velocity */
        }
    }
    else
    {
        if (h->status == MC_IF_HOME_RUNNING) { h->status = MC_IF_HOME_IDLE; }  /* off/preempt -> abort */
        if (in->clear)                       { h->status = MC_IF_HOME_IDLE; }  /* 0 clears done/failed */
    }

    if (h->status == MC_IF_HOME_RUNNING)
    {
        out->active = true;   /* the arbiter owns this tick, incl. the tick a FAILED/DONE transition lands */
        h->total_ticks++;

        if (!h->backing_off)
        {
            /* APPROACH: drive to the stop. Detected by no-movement (armed on "has moved") OR OC trip. */
            const float vmag = fabsf(in->mech_velocity_rad_s);
            if (vmag > MC_HOME_STILL_EPS) { h->moved = true; h->still_ticks = 0u; }
            else if (h->moved)            { h->still_ticks++; }

            if ((h->still_ticks >= MC_HOME_STILL_MS) || in->oc_trip)
            {
                /* Stop found -> capture the datum HERE, then back off before finishing. */
                out->capture_zero = true;
                if (in->oc_trip) { out->consume_oc_trip = true; }   /* consume the trip so back-off can drive */
                h->backing_off   = true;
                h->backoff_ticks = 0u;
                out->reset_slew   = true;   /* ramp the back-off from rest */
                out->want_drive   = true;
                out->velocity_cmd = 0.0f;   /* raw 0 this tick; the ramp begins next tick, in back-off */
                out->slew         = false;
            }
            else if (h->total_ticks >= MC_HOME_TIMEOUT_MS)  /* safety timeout -> fail (no stop found) */
            {
                h->status = MC_IF_HOME_FAILED;
                out->want_drive = false;
            }
            else                                            /* keep driving toward the stop */
            {
                out->want_drive   = true;
                out->velocity_cmd = in->home_velocity_rad_s;   /* ramped by the caller's slew limiter */
                out->slew         = true;
            }
        }
        else
        {
            /* BACK OFF the OPPOSITE direction (ramped) for the fixed dwell, then finish. */
            h->backoff_ticks++;
            if (h->backoff_ticks >= MC_HOME_BACKOFF_MS)
            {
                h->status       = MC_IF_HOME_DONE;
                out->completed  = true;   /* caller: set homed + persist the zero captured at the stop */
                out->want_drive = false;
            }
            else
            {
                out->want_drive   = true;
                out->velocity_cmd = -in->home_velocity_rad_s;
                out->slew         = true;
            }
        }
    }

    out->status = h->status;
}
