#include "mc_scheduler.h"
#include "mc_debug.h"
#include "mc_config.h"
#include "mc_current_sense.h"
#include "mc_ssi_encoder.h"
#include "mc_state_estimator.h"
#include "mc_motor_model.h"
#include "mc_math.h"
#include "mc_pwm.h"
#include <math.h>

/** @file mc_scheduler.c
 *  @brief Timing-domain dispatch (HAL-free). See ADR-006.
 *
 *  Stage A1: the loop bodies are empty placeholders — this stage proves the cadence and
 *  stands up the watch/inject harness with the power stage in safe-off. Subsequent stages
 *  fill in MC_FastLoop_20kHz / MC_MotionLoop_1kHz / MC_SlowLoop_10_100Hz.
 */

/* Slow-loop hand-off flag: set in the medium ISR, consumed in the main loop. */
static volatile uint8_t s_slow_pending;

/* Decimation + timing state (each field written by a single context only). */
static uint8_t  s_slow_div;   /* 1 kHz medium -> /10 -> 100 Hz slow loop */
static uint32_t s_fast_prev;  /* previous fast entry timestamp [cycles] */
static uint32_t s_med_prev;   /* previous medium entry timestamp [cycles] */

/* Stage B1: current-sense instance + latest phase currents. */
static MC_CurrentSense_t  s_cs;
static MC_PhaseCurrents_t s_currents;

/* Stage B2: SSI encoder + state estimator. */
static MC_SsiEncoder_t           s_enc;
static MC_SsiEncoderConfig_t     s_enc_cfg;
static MC_StateEstimator_t       s_est;
static MC_StateEstimatorConfig_t s_est_cfg;
static MC_PositionSensorSample_t s_pos_sample;

/* Stage C2: open-loop drive state. */
#define MC_C2_VD_MAX 3.0f   /* hard clamp on commanded d-axis voltage [V] */
static bool s_oc_trip;      /* latched over-current trip */
static bool s_pwm_on;       /* PWM outputs currently enabled */

void MC_Framework_Init(void)
{
    MC_Debug_Init();
    MC_CurrentSense_Init(&s_cs);

    MC_SsiEncoder_LoadDefaultConfig(&s_enc_cfg);
    MC_SsiEncoder_Init(&s_enc, &s_enc_cfg);

    {
        MC_MotorModel_t motor;
        MC_MotorModel_LoadDefault(&motor);
        s_est_cfg.pole_pairs            = (float)motor.pole_pairs;
        s_est_cfg.electrical_offset_rad = 0.0f;
        s_est_cfg.velocity_filter_hz    = 20.0f;
        s_est_cfg.sample_period_s       = MC_MOTION_DT_S;
        s_est_cfg.obs_kp                = 40000.0f;  /* omega_n = sqrt(kp) = 200 rad/s */
        s_est_cfg.obs_ki                = 0.0f;
        s_est_cfg.obs_kv                = 200.0f;    /* zeta = kv/(2*sqrt(kp)) = 0.5 */
        s_est_cfg.obs_filter_alpha      = 0.3f;
        s_est_cfg.use_observer          = true;      /* ADR-003 default */
    }
    MC_StateEstimator_Init(&s_est);

    /* Seed the live observer-tuning knobs (watch-window writable; not gated, no drive). */
    g_mc_inject.obs_kp = s_est_cfg.obs_kp;
    g_mc_inject.obs_ki = s_est_cfg.obs_ki;
    g_mc_inject.obs_kv = s_est_cfg.obs_kv;
    g_mc_inject.use_finite_diff_velocity = false;

    /* C2 drive defaults (drive stays off until inject_enable is set). */
    g_mc_inject.vbus_v          = 24.0f;   /* set to your actual supply voltage */
    g_mc_inject.current_limit_a = 2.0f;    /* over-current trip [A] */

    g_mc_debug.pwm_enabled = false;   /* power stage starts in safe-off */
}

void MC_Sched_FastTick(void)
{
    /* Called once per PWM period (20 kHz) from the ADC end-of-conversion ISR. The ADC is
       hardware-triggered by TIM1 TRGO=OC4REF at the counter peak (low-side conducting) --
       the proven, sample-synchronised FOC trigger (see ADR-006). */
    uint32_t t0 = MC_Debug_Cycles();
    g_mc_debug.fast_period_cycles = t0 - s_fast_prev;
    s_fast_prev = t0;

    MC_FastLoop_20kHz();

    uint32_t dt = MC_Debug_Cycles() - t0;
    g_mc_debug.fast_cycles = dt;
    if (dt > g_mc_debug.fast_cycles_max)
    {
        g_mc_debug.fast_cycles_max = dt;
    }
    if (dt > g_mc_debug.fast_period_cycles)
    {
        g_mc_debug.fast_overrun_count++;
    }
    g_mc_debug.fast_count++;
}

void MC_Sched_MediumTick(void)
{
    uint32_t t0 = MC_Debug_Cycles();
    g_mc_debug.medium_period_cycles = t0 - s_med_prev;
    s_med_prev = t0;

    MC_MotionLoop_1kHz();

    uint32_t dt = MC_Debug_Cycles() - t0;
    g_mc_debug.medium_cycles = dt;
    if (dt > g_mc_debug.medium_cycles_max)
    {
        g_mc_debug.medium_cycles_max = dt;
    }
    g_mc_debug.medium_count++;

    /* Slow-loop decimation: 1 kHz / 10 = 100 Hz, serviced in the main loop. */
    if (++s_slow_div >= 10u)
    {
        s_slow_div = 0u;
        if (s_slow_pending)
        {
            g_mc_debug.slow_missed_count++;   /* previous slow tick not yet serviced */
        }
        s_slow_pending = 1u;
    }
}

void MC_Sched_ServiceBackground(void)
{
    if (!s_slow_pending)
    {
        return;
    }
    s_slow_pending = 0u;

    uint32_t t0 = MC_Debug_Cycles();
    MC_SlowLoop_10_100Hz();
    g_mc_debug.slow_cycles = MC_Debug_Cycles() - t0;
    g_mc_debug.slow_count++;
}

/* ----- Loop bodies (Stage A1: cadence only; filled in by later stages) ----- */

void MC_FastLoop_20kHz(void)
{
    /* Stage B1: read phase currents (sample-synchronised to the PWM peak). Offset
       calibration runs on request with the power stage in safe-off (no current). */
    if (g_mc_inject.request_offset_cal)
    {
        if (MC_CurrentSense_CalibrateOffsets(&s_cs, 2000u))
        {
            g_mc_inject.request_offset_cal = false;
        }
    }
    else
    {
        (void)MC_CurrentSense_ReadFast(&s_cs, &s_currents);
    }

    /* Mirror to the watch window. */
    g_mc_debug.ia_a               = s_currents.ia_a;
    g_mc_debug.ib_a               = s_currents.ib_a;
    g_mc_debug.ic_a               = s_currents.ic_a;
    g_mc_debug.ia_raw             = s_cs.last_raw_a;
    g_mc_debug.ic_raw             = s_cs.last_raw_c;
    g_mc_debug.ia_offset          = s_cs.offset_a_counts;
    g_mc_debug.ic_offset          = s_cs.offset_c_counts;
    g_mc_debug.current_valid      = s_currents.valid;
    g_mc_debug.current_calibrated = s_cs.calibrated;

    /* Stage C2: over-current monitor + open-loop d-axis voltage (DRIVE gated by inject_enable). */
    float imax = fabsf(s_currents.ia_a);
    const float aib = fabsf(s_currents.ib_a);
    const float aic = fabsf(s_currents.ic_a);
    if (aib > imax) { imax = aib; }
    if (aic > imax) { imax = aic; }
    g_mc_debug.i_max_a = imax;

    if (g_mc_inject.clear_fault) { s_oc_trip = false; g_mc_inject.clear_fault = false; }
    if (s_currents.valid && (imax > g_mc_inject.current_limit_a)) { s_oc_trip = true; }

    const float vd = MC_Math_Clamp(g_mc_inject.align_voltage_v, -MC_C2_VD_MAX, MC_C2_VD_MAX);
    const bool drive = g_mc_inject.inject_enable && !s_oc_trip
                       && !g_mc_inject.request_offset_cal && (vd != 0.0f);
    if (drive)
    {
        float sin_e, cos_e;
        MC_Math_SinCos(g_mc_inject.align_angle_rad, &sin_e, &cos_e);
        const float v_alpha = vd * cos_e;   /* Vq = 0 */
        const float v_beta  = vd * sin_e;
        const float v_a = v_alpha;
        const float v_b = -0.5f * v_alpha + 0.86602540f * v_beta;   /* inverse Clarke */
        const float v_c = -0.5f * v_alpha - 0.86602540f * v_beta;
        const float vbus = (g_mc_inject.vbus_v > 1.0f) ? g_mc_inject.vbus_v : 24.0f;

        MC_PwmDuty_t duty;
        duty.duty_a = 0.5f + (v_a / vbus);
        duty.duty_b = 0.5f + (v_b / vbus);
        duty.duty_c = 0.5f + (v_c / vbus);
        duty.enable = true;

        if (!s_pwm_on) { MC_Pwm_Start(); s_pwm_on = true; }
        MC_Pwm_SetDutyFast(&duty);
        g_mc_debug.pwm_enabled  = true;
        g_mc_debug.vd_applied_v = vd;
    }
    else
    {
        if (s_pwm_on) { MC_Pwm_ForceSafeOff(); s_pwm_on = false; }
        g_mc_debug.pwm_enabled  = false;
        g_mc_debug.vd_applied_v = 0.0f;
    }
    g_mc_debug.overcurrent_trip = s_oc_trip;
}

void MC_MotionLoop_1kHz(void)
{
    /* Stage B2: read the SSI encoder and update the state estimator. */
    /* Apply live observer tuning + velocity-source selection from the watch window. */
    s_est_cfg.obs_kp       = g_mc_inject.obs_kp;
    s_est_cfg.obs_ki       = g_mc_inject.obs_ki;
    s_est_cfg.obs_kv       = g_mc_inject.obs_kv;
    s_est_cfg.use_observer = !g_mc_inject.use_finite_diff_velocity;

    if (MC_SsiEncoder_ReadHardware(&s_enc, &s_enc_cfg, &s_pos_sample))
    {
        MC_StateEstimator_Update(&s_est, &s_est_cfg, &s_pos_sample);
    }

    /* Mirror to the watch window. */
    g_mc_debug.enc_raw             = s_pos_sample.raw_position;
    g_mc_debug.mech_position_rad   = s_est.mechanical.position_rad;
    g_mc_debug.mech_velocity_rad_s = s_est.mechanical.velocity_rad_per_s;  /* active source */
    g_mc_debug.vel_finite_diff     = s_est.velocity_filtered;
    g_mc_debug.vel_observer        = s_est.velocity_observer;
    g_mc_debug.elec_angle_rad      = s_est.electrical.electrical_angle_rad;
    g_mc_debug.enc_valid           = s_pos_sample.valid;

    /* Stage C2: capture the electrical offset so the electrical angle reads 0 at the held
       rotor position (commanded electrical angle 0). */
    if (g_mc_inject.request_align_capture)
    {
        g_mc_inject.request_align_capture = false;
        s_est_cfg.electrical_offset_rad =
            MC_Math_Wrap2Pi(-s_pos_sample.position_rad * s_est_cfg.pole_pairs);
        g_mc_debug.elec_offset_rad = s_est_cfg.electrical_offset_rad;
    }
}

void MC_SlowLoop_10_100Hz(void)
{
    /* Stage A1: no supervisory work yet. */
}
