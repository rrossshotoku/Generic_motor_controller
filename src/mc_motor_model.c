#include "mc_motor_model.h"

/** @file mc_motor_model.c
 *  @brief Default motor model (Maxon EC 90 flat, part 500267). See ADR-004.
 */

void MC_MotorModel_LoadDefault(MC_MotorModel_t *model)
{
    model->backend_type = MC_MOTOR_BACKEND_BLDC_FOC_3SHUNT;
    model->pole_pairs   = 11u;
    model->phase_count  = 3u;

    model->resistance_ohm     = 0.844f;
    model->inductance_h       = 0.00107f;
    model->kt_nm_per_a        = 0.231f;
    model->ke_v_per_rad_per_s = 0.231f;

    model->rotor_inertia_kg_m2           = 0.000506f;
    model->friction_coulomb_nm           = 0.0f;
    model->friction_viscous_nm_s_per_rad = 0.0f;

    model->nominal_voltage_v   = 24.0f;
    model->nominal_current_a   = 4.06f;
    model->nominal_torque_nm   = 0.964f;
    model->stall_current_a     = 56.9f;
    model->stall_torque_nm     = 9.41f;
    model->max_speed_rad_per_s = 523.6f;   /* 5000 rpm */

    model->thermal_time_constant_s   = 54.3f;
    model->thermal_rise_at_nominal_c = 100.0f;
    model->winding_temp_max_c        = 125.0f;
}

MC_Status_t MC_MotorModel_Validate(const MC_MotorModel_t *model)
{
    if (model == 0)
    {
        return MC_ERR_INVALID_ARG;
    }
    if ((model->pole_pairs == 0u) ||
        (model->kt_nm_per_a <= 0.0f) ||
        (model->rotor_inertia_kg_m2 <= 0.0f) ||
        (model->resistance_ohm <= 0.0f))
    {
        return MC_ERR_RANGE;
    }
    return MC_OK;
}
