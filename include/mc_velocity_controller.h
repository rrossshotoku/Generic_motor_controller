#ifndef MC_VELOCITY_CONTROLLER_H
#define MC_VELOCITY_CONTROLLER_H
#include "mc_types.h"
#include "mc_pid.h"

/** @ingroup mc_control */
typedef struct
{
    MC_Pid_t pid;
    float torque_correction_nm;
    float velocity_error_rad_per_s;
    bool enabled;
} MC_VelocityController_t;

typedef struct
{
    MC_PidConfig_t pid;
    float torque_output_limit_nm;
    float velocity_error_limit_rad_per_s;
    float stop_bleed_v_th;    /**< Stop-integrator bleed (ADR-074): fast-unwind the integrator when commanded
                                   to stop (demand~0) and |actual| below this [rad/s], so the wound-up brake
                                   can't push velocity past zero into a reverse. 0 = disabled. */
    float stop_bleed_rate;    /**< Integrator unwind rate [1/s] while bleeding (0 = disabled). */
} MC_VelocityControllerConfig_t;

void MC_VelocityController_Init(MC_VelocityController_t *ctrl);
void MC_VelocityController_Reset(MC_VelocityController_t *ctrl);
float MC_VelocityController_Update(MC_VelocityController_t *ctrl,
                                   const MC_VelocityControllerConfig_t *cfg,
                                   float velocity_demand_rad_per_s,
                                   float velocity_actual_rad_per_s);

#endif
