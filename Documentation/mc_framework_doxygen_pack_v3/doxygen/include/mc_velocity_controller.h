#ifndef MC_VELOCITY_CONTROLLER_H
#define MC_VELOCITY_CONTROLLER_H
#include "mc_types.h"
#include "mc_pid.h"
typedef struct { MC_Pid_t pid; } MC_VelocityController_t;
float MC_VelocityController_Update(MC_VelocityController_t *ctrl, const MC_PidConfig_t *cfg, float velocity_demand_rad_per_s, float velocity_actual_rad_per_s);
#endif
