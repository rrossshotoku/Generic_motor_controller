#ifndef MC_POSITION_CONTROLLER_H
#define MC_POSITION_CONTROLLER_H
#include "mc_types.h"
#include "mc_pid.h"
typedef struct { MC_Pid_t pid; } MC_PositionController_t;
float MC_PositionController_Update(MC_PositionController_t *ctrl, const MC_PidConfig_t *cfg, float position_demand_rad, float position_actual_rad);
#endif
