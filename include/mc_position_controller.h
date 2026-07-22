#ifndef MC_POSITION_CONTROLLER_H
#define MC_POSITION_CONTROLLER_H
#include "mc_types.h"
#include "mc_pid.h"

/** @ingroup mc_control */
typedef struct
{
    MC_Pid_t pid;
    float velocity_correction_rad_per_s;
    float position_error_rad;
    bool enabled;
} MC_PositionController_t;

typedef struct
{
    MC_PidConfig_t pid;
    float velocity_correction_limit_rad_per_s;
    float following_error_limit_rad;
    float deadband_rad;   /**< Position-error deadband [rad]: no correction within +/- this of target; 0 = off (ADR-071). */
} MC_PositionControllerConfig_t;

void MC_PositionController_Init(MC_PositionController_t *ctrl);
void MC_PositionController_Reset(MC_PositionController_t *ctrl);
float MC_PositionController_Update(MC_PositionController_t *ctrl,
                                   const MC_PositionControllerConfig_t *cfg,
                                   float position_demand_rad,
                                   float position_actual_rad);

#endif
