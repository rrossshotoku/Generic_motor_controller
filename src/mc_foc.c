#include "mc_foc.h"
#include "mc_math.h"

/** @file mc_foc.c
 *  @brief BLDC/PMSM FOC current controller (three-shunt). See ADR-011.
 *
 *  Clarke (amplitude-invariant) -> Park -> d/q PI -> circular voltage limit -> inverse Park ->
 *  SVPWM (inverse Clarke + min/max zero-sequence injection, duty clamp 0.95). Ported from the
 *  proven bldc_axis_controller FOC. TIM/ADC stay outside this module.
 */

#define MC_FOC_SQRT3_INV  0.57735026918962576f   /* 1/sqrt(3) */
#define MC_FOC_SQRT3_HALF 0.86602540378443865f   /* sqrt(3)/2 */

void MC_Foc_Init(MC_Foc_t *foc, const MC_FocConfig_t *cfg)
{
    (void)cfg;
    MC_Pid_Init(&foc->id_loop);
    MC_Pid_Init(&foc->iq_loop);
    foc->id_measured_a     = 0.0f;
    foc->iq_measured_a     = 0.0f;
    foc->vd_v              = 0.0f;
    foc->vq_v              = 0.0f;
    foc->voltage_saturated = false;
    foc->enabled           = false;
}

void MC_Foc_Reset(MC_Foc_t *foc)
{
    MC_Pid_Reset(&foc->id_loop);
    MC_Pid_Reset(&foc->iq_loop);
    foc->vd_v              = 0.0f;
    foc->vq_v              = 0.0f;
    foc->voltage_saturated = false;
}

MC_PwmDuty_t MC_Foc_Update(MC_Foc_t *foc,
                           const MC_FocConfig_t *cfg,
                           const MC_FocCurrentCommand_t *cmd,
                           const MC_PhaseCurrents_t *currents,
                           const MC_ElectricalState_t *electrical,
                           float bus_voltage_v)
{
    float sin_e, cos_e;
    MC_Math_SinCos(electrical->electrical_angle_rad, &sin_e, &cos_e);

    /* Clarke (amplitude-invariant): ialpha = ia; ibeta = (ia + 2*ib)/sqrt3. */
    const float ia = currents->ia_a;
    const float ib = currents->ib_a;
    const float i_alpha = ia;
    const float i_beta  = (ia + 2.0f * ib) * MC_FOC_SQRT3_INV;

    /* Park: id = ialpha*cos + ibeta*sin; iq = -ialpha*sin + ibeta*cos. */
    const float id =  i_alpha * cos_e + i_beta * sin_e;
    const float iq = -i_alpha * sin_e + i_beta * cos_e;
    foc->id_measured_a = id;
    foc->iq_measured_a = iq;

    /* d/q PI current loops (setpoint - measurement inside). */
    float vd = MC_Pid_Update(&foc->id_loop, &cfg->id_pi, cmd->id_a, id);
    float vq = MC_Pid_Update(&foc->iq_loop, &cfg->iq_pi, cmd->iq_a, iq);

    /* Circular voltage-vector limit. */
    const float vlim = cfg->voltage_limit_v;
    const float vmag = MC_Math_Sqrt(vd * vd + vq * vq);
    if ((vlim > 0.0f) && (vmag > vlim))
    {
        const float scale = vlim / vmag;
        vd *= scale;
        vq *= scale;
        foc->voltage_saturated = true;
    }
    else
    {
        foc->voltage_saturated = false;
    }
    foc->vd_v = vd;
    foc->vq_v = vq;

    /* Inverse Park: valpha = vd*cos - vq*sin; vbeta = vd*sin + vq*cos. */
    const float v_alpha = vd * cos_e - vq * sin_e;
    const float v_beta  = vd * sin_e + vq * cos_e;

    /* SVPWM: per-unit inverse Clarke + min/max zero-sequence injection. */
    const float vbus = (bus_voltage_v > 1.0f) ? bus_voltage_v : 1.0f;
    const float ua = v_alpha / vbus;
    const float ub = -0.5f * (v_alpha / vbus) + MC_FOC_SQRT3_HALF * (v_beta / vbus);
    const float uc = -0.5f * (v_alpha / vbus) - MC_FOC_SQRT3_HALF * (v_beta / vbus);

    float umax = ua;
    float umin = ua;
    if (ub > umax) { umax = ub; }
    if (uc > umax) { umax = uc; }
    if (ub < umin) { umin = ub; }
    if (uc < umin) { umin = uc; }
    const float uoff = 0.5f * (umax + umin);

    MC_PwmDuty_t duty;
    duty.duty_a = MC_Math_Clamp(0.5f + (ua - uoff), 0.0f, 0.95f);
    duty.duty_b = MC_Math_Clamp(0.5f + (ub - uoff), 0.0f, 0.95f);
    duty.duty_c = MC_Math_Clamp(0.5f + (uc - uoff), 0.0f, 0.95f);
    duty.enable = cmd->enable;
    return duty;
}
