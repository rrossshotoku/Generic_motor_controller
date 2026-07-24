#include "mc_current_request.h"
#include "mc_math.h"

/** @file mc_current_request.c
 *  @brief Torque/current request generator. See ADR-012.
 *
 *  Combines the velocity-loop torque correction with acceleration and friction feedforward,
 *  applies the torque limit, and converts the resulting torque to a FOC current command via the
 *  torque constant (motor-agnostic seam: outer layers speak torque/current, not iq). For a
 *  future brushed-DC backend the same torque request maps to armature current instead.
 */

MC_MotorTorqueRequest_t MC_CurrentRequest_Update(const MC_TorqueModelConfig_t *cfg,
                                                 float velocity_feedback_correction_nm,
                                                 float acceleration_ff_rad_per_s2,
                                                 float velocity_rad_per_s,
                                                 bool enable,
                                                 MC_CurrentRequestDebug_t *dbg)
{
    const float t_accel = cfg->accel_ff_gain * cfg->inertia_kg_m2 * acceleration_ff_rad_per_s2;  /* gain 0x2300:15 (ADR-079) */
    const float t_fric  = cfg->static_friction_nm * MC_Math_Sign(velocity_rad_per_s)
                        + cfg->viscous_friction_nm_per_rad_s * velocity_rad_per_s;
    const float t_raw   = velocity_feedback_correction_nm + t_accel + t_fric;
    const float t_lim   = MC_Math_Clamp(t_raw, -cfg->torque_limit_nm, cfg->torque_limit_nm);

    if (dbg != 0)
    {
        dbg->torque_feedback_nm    = velocity_feedback_correction_nm;
        dbg->torque_accel_ff_nm    = t_accel;
        dbg->torque_friction_ff_nm = t_fric;
        dbg->torque_limited_nm     = t_lim;
        dbg->limited               = (t_lim != t_raw);
    }

    MC_MotorTorqueRequest_t req;
    req.torque_nm       = t_lim;
    req.current_limit_a = cfg->current_limit_a;
    req.enable          = enable;
    return req;
}

MC_FocCurrentCommand_t MC_CurrentRequest_ToFocCommand(const MC_TorqueModelConfig_t *cfg,
                                                      const MC_MotorTorqueRequest_t *request)
{
    MC_FocCurrentCommand_t cmd;
    float iq = (cfg->torque_constant_nm_per_a > 1.0e-6f)
             ? (request->torque_nm / cfg->torque_constant_nm_per_a)
             : 0.0f;
    iq = MC_Math_Clamp(iq, -request->current_limit_a, request->current_limit_a);
    cmd.id_a   = 0.0f;
    cmd.iq_a   = iq;
    cmd.enable = request->enable;
    return cmd;
}
