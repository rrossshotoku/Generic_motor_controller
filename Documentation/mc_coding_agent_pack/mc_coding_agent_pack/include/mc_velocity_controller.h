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
} MC_VelocityControllerConfig_t;

void MC_VelocityController_Init(MC_VelocityController_t *ctrl);
void MC_VelocityController_Reset(MC_VelocityController_t *ctrl);
float MC_VelocityController_Update(MC_VelocityController_t *ctrl,
                                   const MC_VelocityControllerConfig_t *cfg,
                                   float velocity_demand_rad_per_s,
                                   float velocity_actual_rad_per_s);

#endif
