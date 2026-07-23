#include "mc_velocity_controller.h"
#include <math.h>

/** @file mc_velocity_controller.c
 *  @brief Velocity controller: PI on velocity error -> torque correction (Nm). See ADR-012.
 *
 *  Outputs a torque request (motor-agnostic); the current-request generator converts torque to
 *  the FOC iq command via the torque constant. Default behaviour ports the proven velocity loop
 *  (gains expressed in torque rather than current; net iq identical).
 */

#define MC_VEL_STOP_DEMAND_EPS (1.0e-3f)   /* |demand| below this counts as "commanded to stop" (ADR-074) */

void MC_VelocityController_Init(MC_VelocityController_t *ctrl)
{
    MC_Pid_Init(&ctrl->pid);
    ctrl->torque_correction_nm     = 0.0f;
    ctrl->velocity_error_rad_per_s = 0.0f;
    ctrl->enabled                  = true;
}

void MC_VelocityController_Reset(MC_VelocityController_t *ctrl)
{
    MC_Pid_Reset(&ctrl->pid);
    ctrl->torque_correction_nm     = 0.0f;
    ctrl->velocity_error_rad_per_s = 0.0f;
}

float MC_VelocityController_Update(MC_VelocityController_t *ctrl,
                                   const MC_VelocityControllerConfig_t *cfg,
                                   float velocity_demand_rad_per_s,
                                   float velocity_actual_rad_per_s)
{
    ctrl->velocity_error_rad_per_s = velocity_demand_rad_per_s - velocity_actual_rad_per_s;

    /* Stop-integrator bleed (ADR-074). When commanded to stop (demand ~ 0) and actually slow
       (|actual| < v_th), fast-unwind the integrator BEFORE this tick's PI runs -- its wound-up
       braking is what pushes the velocity past zero into a reverse (the on-camera recoil). The
       proportional term alone brakes to rest without overshoot, so draining the integral gives a
       clean stop. First-order decay at `rate` [1/s]; disabled when v_th or rate is 0. Gated on the
       demand, so it engages firmly on a jog stop (demand parks at 0) but only fleetingly at a
       shot-recall landing (demand kisses 0 then goes negative to correct) -- leaving the position
       loop free to land on target. */
    if (cfg->stop_bleed_enable &&
        (cfg->stop_bleed_v_th > 0.0f) && (cfg->stop_bleed_factor > 0.0f) &&
        (fabsf(velocity_demand_rad_per_s) < MC_VEL_STOP_DEMAND_EPS) &&
        (fabsf(velocity_actual_rad_per_s) < cfg->stop_bleed_v_th))
    {
        /* Unwind speed as a factor of ki: per-tick fraction removed = factor*ki*dt (2 = twice as
           fast). Ties the bleed to the windup timescale + auto-disables when ki = 0 (no integral). */
        float k = cfg->stop_bleed_factor * cfg->pid.ki * cfg->pid.sample_period_s;
        if (k > 1.0f)      { k = 1.0f; }
        else if (k < 0.0f) { k = 0.0f; }
        ctrl->pid.integrator -= ctrl->pid.integrator * k;
    }

    /* Derivative-on-measurement (configured) avoids a kick on demand steps. */
    ctrl->torque_correction_nm =
        MC_Pid_Update(&ctrl->pid, &cfg->pid, velocity_demand_rad_per_s, velocity_actual_rad_per_s);
    return ctrl->torque_correction_nm;
}
