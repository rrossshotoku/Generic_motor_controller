#include "mc_board_config.h"

/** @file mc_board_config.c
 *  @brief Board profile 0 (the proven reference board) and current-sense scaling helpers.
 *         See ADR-004 and docs/spec/17_board_and_motor_config.md.
 */

void MC_BoardConfig_LoadProfile0(MC_BoardConfig_t *cfg)
{
    cfg->name = "reference_board_profile_0";

    /* Phase-current sensing (known-good from bldc_axis_controller). */
    cfg->current_sense.shunt_resistance_ohm  = 0.01f;
    cfg->current_sense.amp_gain              = 5.18f;
    cfg->current_sense.zero_current_offset_v = 1.71f;
    cfg->current_sense.adc_vref_v            = 3.3f;
    cfg->current_sense.adc_full_scale        = 4096u;
    cfg->current_sense.phase_sign[0]         = -1;   /* convention: positive = current into phase */
    cfg->current_sense.phase_sign[1]         = -1;
    cfg->current_sense.phase_sign[2]         = -1;

    /* DC-bus sense: the reference firmware does not convert Vbus -- BOARD VALUE, confirm. */
    cfg->bus_sense.divider_ratio  = 0.0f;
    cfg->bus_sense.adc_vref_v     = 3.3f;
    cfg->bus_sense.adc_full_scale = 4096u;

    /* Temperature: no analog sensor wired on profile 0; use the motor thermal model. */
    cfg->temp_sense.analog_sensor_present = false;
    cfg->temp_sense.scale_c_per_count     = 0.0f;
    cfg->temp_sense.offset_c              = 0.0f;

    /* PWM power stage (TIM1, centre-aligned). */
    cfg->pwm.frequency_hz          = 20000u;
    cfg->pwm.period_counts         = 4250u;
    cfg->pwm.deadtime_counts       = 200u;   /* confirmed from the TIM1 config */
    cfg->pwm.complementary_outputs = true;
    cfg->pwm.active_high           = true;

    /* Position feedback (SSI absolute, 21-bit). */
    cfg->encoder.backend                    = MC_ENCODER_BACKEND_SSI;
    cfg->encoder.counts_per_rev             = 2097152u;   /* 2^21 */
    cfg->encoder.resolution_bits            = 21u;
    cfg->encoder.invert_direction           = true;
    cfg->encoder.mechanical_zero_offset_rad = 0.0f;

    /* CubeMX handles bound by the boundary modules at init. */
    cfg->htim_pwm     = 0;
    cfg->hadc_phase_a = 0;
    cfg->hadc_phase_c = 0;
    cfg->hadc_vbus    = 0;
    cfg->hspi_encoder = 0;
    cfg->hspi_link    = 0;
}

float MC_CurrentSense_AmpsPerCount(const MC_CurrentSenseConfig_t *cs)
{
    return cs->adc_vref_v /
           ((float)cs->adc_full_scale * cs->amp_gain * cs->shunt_resistance_ohm);
}

uint16_t MC_CurrentSense_ZeroCount(const MC_CurrentSenseConfig_t *cs)
{
    float counts = (cs->zero_current_offset_v / cs->adc_vref_v) * (float)cs->adc_full_scale;
    if (counts < 0.0f)
    {
        counts = 0.0f;
    }
    return (uint16_t)(counts + 0.5f);   /* round to nearest */
}

MC_Status_t MC_BoardConfig_Validate(const MC_BoardConfig_t *cfg)
{
    if (cfg == 0)
    {
        return MC_ERR_INVALID_ARG;
    }
    if ((cfg->current_sense.shunt_resistance_ohm <= 0.0f) ||
        (cfg->current_sense.amp_gain <= 0.0f) ||
        (cfg->current_sense.adc_vref_v <= 0.0f) ||
        (cfg->current_sense.adc_full_scale == 0u) ||
        (cfg->pwm.period_counts == 0u))
    {
        return MC_ERR_RANGE;
    }
    return MC_OK;
}
