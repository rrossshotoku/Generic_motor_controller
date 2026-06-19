#include "mc_velocity_controller.h"
#include "mc_math.h"
void MC_VelocityController_Init(MC_VelocityController_t *ctrl) { if (ctrl) { *ctrl = (MC_VelocityController_t){0}; MC_Pid_Init(&ctrl->pid); ctrl->enabled = true; } }
void MC_VelocityController_Reset(MC_VelocityController_t *ctrl) { if (ctrl) { MC_Pid_Reset(&ctrl->pid); ctrl->torque_correction_nm = 0.0f; ctrl->velocity_error_rad_per_s = 0.0f; } }
float MC_VelocityController_Update(MC_VelocityController_t *ctrl, const MC_VelocityControllerConfig_t *cfg, float demand, float actual) { if (!ctrl || !cfg || !ctrl->enabled) return 0.0f; ctrl->velocity_error_rad_per_s = demand - actual; ctrl->torque_correction_nm = MC_Pid_UpdateError(&ctrl->pid, &cfg->pid, ctrl->velocity_error_rad_per_s, actual); ctrl->torque_correction_nm = MC_Math_Clamp(ctrl->torque_correction_nm, -cfg->torque_output_limit_nm, cfg->torque_output_limit_nm); return ctrl->torque_correction_nm; }
