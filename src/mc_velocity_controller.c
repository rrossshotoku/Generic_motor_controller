#include "mc_velocity_controller.h"

/** @file mc_velocity_controller.c
 *  @brief Velocity controller: PI on velocity error -> torque correction (Nm). See ADR-012.
 *
 *  Outputs a torque request (motor-agnostic); the current-request generator converts torque to
 *  the FOC iq command via the torque constant. Default behaviour ports the proven velocity loop
 *  (gains expressed in torque rather than current; net iq identical).
 */

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
    /* Derivative-on-measurement (configured) avoids a kick on demand steps. */
    ctrl->torque_correction_nm =
        MC_Pid_Update(&ctrl->pid, &cfg->pid, velocity_demand_rad_per_s, velocity_actual_rad_per_s);
    return ctrl->torque_correction_nm;
}
