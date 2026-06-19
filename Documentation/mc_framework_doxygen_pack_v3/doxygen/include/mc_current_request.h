#ifndef MC_CURRENT_REQUEST_H
#define MC_CURRENT_REQUEST_H
#include "mc_types.h"

typedef struct {
    float inertia_kg_m2;
    float torque_constant_nm_per_a;
    float static_friction_nm;
    float viscous_friction_nm_per_rad_s;
    float current_limit_a;
} MC_TorqueModelConfig_t;

MC_MotorTorqueRequest_t MC_CurrentRequest_Update(const MC_TorqueModelConfig_t *cfg,
                                                 float velocity_feedback_correction_nm,
                                                 float acceleration_ff_rad_per_s2,
                                                 float velocity_rad_per_s,
                                                 bool enable);
MC_FocCurrentCommand_t MC_CurrentRequest_ToFocCommand(const MC_TorqueModelConfig_t *cfg,
                                                      const MC_MotorTorqueRequest_t *request);
#endif
