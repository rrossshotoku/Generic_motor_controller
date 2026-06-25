#include "mc_position_controller.h"

/** @file mc_position_controller.c
 *  @brief Position controller: P/PID on position error -> velocity correction (rad/s). ADR-028, spec 09.
 *
 *  velocity_correction = PID(position_demand - position_actual), P-only by default (gains 0x2200).
 *  The scheduler forms velocity_demand = trajectory_velocity_ff + velocity_correction and feeds the
 *  velocity loop. The position error is bounded by `following_error_limit_rad` (a guard against a huge
 *  transient correction on a large error, and the basis for a future following-error fault) and the
 *  output is clamped to `velocity_correction_limit_rad_per_s`.
 */

void MC_PositionController_Init(MC_PositionController_t *ctrl)
{
    if (ctrl == 0) { return; }
    MC_Pid_Init(&ctrl->pid);
    ctrl->velocity_correction_rad_per_s = 0.0f;
    ctrl->position_error_rad            = 0.0f;
    ctrl->enabled                       = true;
}

void MC_PositionController_Reset(MC_PositionController_t *ctrl)
{
    if (ctrl == 0) { return; }
    MC_Pid_Reset(&ctrl->pid);
    ctrl->velocity_correction_rad_per_s = 0.0f;
    ctrl->position_error_rad            = 0.0f;
}

float MC_PositionController_Update(MC_PositionController_t *ctrl,
                                   const MC_PositionControllerConfig_t *cfg,
                                   float position_demand_rad,
                                   float position_actual_rad)
{
    if ((ctrl == 0) || (cfg == 0)) { return 0.0f; }

    const float err = position_demand_rad - position_actual_rad;
    ctrl->position_error_rad = err;

    /* Bound the error driving the loop (following-error guard) so a large position step can't demand a
       wild velocity correction; the limit also feeds a future following-error fault. */
    float e = err;
    const float fe = cfg->following_error_limit_rad;
    if (fe > 0.0f)
    {
        if      (e >  fe) { e =  fe; }
        else if (e < -fe) { e = -fe; }
    }

    /* P/PID on the (bounded) error; derivative-on-measurement uses the actual position. */
    float out = MC_Pid_UpdateError(&ctrl->pid, &cfg->pid, e, position_actual_rad);

    const float lim = cfg->velocity_correction_limit_rad_per_s;
    if (lim > 0.0f)
    {
        if      (out >  lim) { out =  lim; }
        else if (out < -lim) { out = -lim; }
    }

    ctrl->velocity_correction_rad_per_s = out;
    return out;
}
