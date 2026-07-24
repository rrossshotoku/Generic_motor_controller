#ifndef MC_CURRENT_REQUEST_H
#define MC_CURRENT_REQUEST_H
#include "mc_types.h"

/** @file mc_current_request.h
 *  @brief Combines velocity feedback correction, acceleration FF, friction and limits.
 *  @ingroup mc_control
 */

typedef struct
{
    float inertia_kg_m2;
    float accel_ff_gain;      /**< Scales the inertia accel feedforward (0x2300:15, ADR-079); analog of the
                                   position loop's velocity_ff_gain. 1.0 = full physical FF, 0 = none. */
    float torque_constant_nm_per_a;
    float static_friction_nm;
    float viscous_friction_nm_per_rad_s;
    float current_limit_a;
    float torque_limit_nm;
} MC_TorqueModelConfig_t;

typedef struct
{
    float torque_feedback_nm;
    float torque_accel_ff_nm;
    float torque_friction_ff_nm;
    float torque_limited_nm;
    bool limited;
} MC_CurrentRequestDebug_t;

MC_MotorTorqueRequest_t MC_CurrentRequest_Update(const MC_TorqueModelConfig_t *cfg,
                                                 float velocity_feedback_correction_nm,
                                                 float acceleration_ff_rad_per_s2,
                                                 float velocity_rad_per_s,
                                                 bool enable,
                                                 MC_CurrentRequestDebug_t *dbg);

MC_FocCurrentCommand_t MC_CurrentRequest_ToFocCommand(const MC_TorqueModelConfig_t *cfg,
                                                      const MC_MotorTorqueRequest_t *request);
#endif
