#include "mc_position_controller.h"
#include "mc_math.h"
void MC_PositionController_Init(MC_PositionController_t *ctrl) { if (ctrl) { *ctrl = (MC_PositionController_t){0}; MC_Pid_Init(&ctrl->pid); ctrl->enabled = true; } }
void MC_PositionController_Reset(MC_PositionController_t *ctrl) { if (ctrl) { MC_Pid_Reset(&ctrl->pid); ctrl->velocity_correction_rad_per_s = 0.0f; ctrl->position_error_rad = 0.0f; } }
float MC_PositionController_Update(MC_PositionController_t *ctrl, const MC_PositionControllerConfig_t *cfg, float demand, float actual) { if (!ctrl || !cfg || !ctrl->enabled) return 0.0f; ctrl->position_error_rad = demand - actual; ctrl->velocity_correction_rad_per_s = MC_Pid_UpdateError(&ctrl->pid, &cfg->pid, ctrl->position_error_rad, actual); ctrl->velocity_correction_rad_per_s = MC_Math_Clamp(ctrl->velocity_correction_rad_per_s, -cfg->velocity_correction_limit_rad_per_s, cfg->velocity_correction_limit_rad_per_s); return ctrl->velocity_correction_rad_per_s; }
