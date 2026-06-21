#include "mc_scheduler.h"
#include "mc_debug.h"
#include "mc_config.h"
#include "mc_current_sense.h"
#include "mc_ssi_encoder.h"
#include "mc_state_estimator.h"
#include "mc_motor_model.h"
#include "mc_math.h"
#include "mc_pwm.h"
#include "mc_foc.h"
#include "mc_persistent_store.h"
#include "mc_calib_data.h"
#include "mc_velocity_controller.h"
#include "mc_current_request.h"
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

/* Stage D1: FOC current loop. */
static MC_Foc_t       s_foc;
static MC_FocConfig_t s_foc_cfg;
static bool           s_foc_on;       /* FOC active (for entry reset) */
static volatile float s_elec_angle;   /* electrical angle published medium->fast (atomic float) */

/* Stage D2: velocity loop + torque/current request (runs in the medium loop). */
static MC_MotorModel_t               s_motor;
static MC_VelocityController_t       s_vel;
static MC_VelocityControllerConfig_t s_vel_cfg;
static MC_TorqueModelConfig_t        s_torque_cfg;
static volatile float                s_iq_cmd_published;  /* velocity-loop iq, medium->fast (atomic) */
static bool                          s_vel_on;            /* velocity loop active (for entry reset) */

/* Build the calibration payload from the live config + latch a flash save (written by the
   slow loop when the power stage is off). See ADR-010. */
static void calib_save(void)
{
    MC_CalibData_t cal;
    cal.electrical_offset_rad      = s_est_cfg.electrical_offset_rad;
    cal.current_offset_a_counts    = s_cs.offset_a_counts;
    cal.current_offset_c_counts    = s_cs.offset_c_counts;
    cal.mechanical_zero_offset_rad = s_enc_cfg.mechanical_zero_offset_rad;
    cal.phase_order                = 1;     /* phase-order detection is Phase E */
    cal.reserved                   = 0u;
    MC_PersistentStore_RequestSave(&cal, (uint16_t)sizeof cal);
}

void MC_Framework_Init(void)
{
    MC_Debug_Init();
    MC_CurrentSense_Init(&s_cs);

    MC_SsiEncoder_LoadDefaultConfig(&s_enc_cfg);
    MC_SsiEncoder_Init(&s_enc, &s_enc_cfg);

    MC_MotorModel_LoadDefault(&s_motor);
    {
        s_est_cfg.pole_pairs            = (float)s_motor.pole_pairs;
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
    g_mc_inject.current_limit_a = 3.0f;    /* over-current trip [A] (headroom over ~1.5-2 A breakaway) */

    /* D1 FOC current-loop config (ported gains; runs in the fast loop). */
    {
        MC_PidConfig_t ipi;
        ipi.kp                        = 1.7f;
        ipi.ki                        = 1700.0f;
        ipi.kd                        = 0.0f;
        ipi.sample_period_s           = MC_FAST_DT_S;
        ipi.output_min                = -24.0f;
        ipi.output_max                =  24.0f;
        ipi.integrator_min            = -24.0f;
        ipi.integrator_max            =  24.0f;
        ipi.derivative_filter_hz      = 0.0f;
        ipi.integrator_enabled        = true;
        ipi.derivative_enabled        = false;
        ipi.derivative_on_measurement = true;
        s_foc_cfg.id_pi                   = ipi;
        s_foc_cfg.iq_pi                   = ipi;
        s_foc_cfg.voltage_limit_v         = 13.8f;   /* ~Vbus/sqrt(3) for linear SVPWM at 24 V */
        s_foc_cfg.sample_period_s         = MC_FAST_DT_S;
        s_foc_cfg.use_cordic_if_available = false;
    }
    MC_Foc_Init(&s_foc, &s_foc_cfg);

    /* D2 velocity loop + torque model. Gains ported from the proven current-output loop,
       expressed as torque = old_gain * Kt (net iq identical); output limit = current * Kt. */
    {
        const float kt        = s_motor.kt_nm_per_a;          /* 0.231 Nm/A */
        const float i_lim     = 2.5f;                         /* velocity-loop current limit [A] */
        const float torque_lim = i_lim * kt;

        s_torque_cfg.inertia_kg_m2                 = s_motor.rotor_inertia_kg_m2;
        s_torque_cfg.torque_constant_nm_per_a      = kt;
        s_torque_cfg.static_friction_nm            = 0.0f;    /* FF off until identified */
        s_torque_cfg.viscous_friction_nm_per_rad_s = 0.0f;
        s_torque_cfg.current_limit_a               = i_lim;
        s_torque_cfg.torque_limit_nm               = torque_lim;

        MC_PidConfig_t vpid;
        vpid.kp                        = 150.0f * kt;         /* ~34.65 Nm/(rad/s) */
        vpid.ki                        = 1000.0f * kt;        /* 231 */
        vpid.kd                        = 0.0f;
        vpid.sample_period_s           = MC_MOTION_DT_S;
        vpid.output_min                = -torque_lim;
        vpid.output_max                =  torque_lim;
        vpid.integrator_min            = -torque_lim;
        vpid.integrator_max            =  torque_lim;
        vpid.derivative_filter_hz      = 0.0f;
        vpid.integrator_enabled        = true;
        vpid.derivative_enabled        = false;
        vpid.derivative_on_measurement = true;
        s_vel_cfg.pid                          = vpid;
        s_vel_cfg.torque_output_limit_nm       = torque_lim;
        s_vel_cfg.velocity_error_limit_rad_per_s = 0.0f;
    }
    MC_VelocityController_Init(&s_vel);

    /* Load persisted calibration (electrical offset + current offsets) if present. */
    if (MC_PersistentStore_Init() == MC_OK)
    {
        MC_CalibData_t cal;
        if (MC_PersistentStore_Read(&cal, (uint16_t)sizeof cal) == MC_OK)
        {
            s_est_cfg.electrical_offset_rad   = cal.electrical_offset_rad;
            s_enc_cfg.mechanical_zero_offset_rad = cal.mechanical_zero_offset_rad;
            s_cs.offset_a_counts              = cal.current_offset_a_counts;
            s_cs.offset_c_counts              = cal.current_offset_c_counts;
            s_cs.calibrated                   = true;
            g_mc_debug.elec_offset_rad        = cal.electrical_offset_rad;
            g_mc_debug.store_valid            = true;
        }
    }

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

    const bool armed = g_mc_inject.inject_enable && !s_oc_trip && !g_mc_inject.request_offset_cal;

    if (armed && g_mc_inject.foc_enable)
    {
        /* Stage D1: closed FOC current loop. */
        if (!s_foc_on) { MC_Foc_Reset(&s_foc); s_foc_on = true; }

        MC_ElectricalState_t elec;
        elec.electrical_angle_rad          = s_elec_angle;   /* published by the medium loop */
        elec.electrical_velocity_rad_per_s = 0.0f;
        elec.electrical_valid              = true;

        MC_FocCurrentCommand_t cmd;
        cmd.id_a   = g_mc_inject.id_cmd_a;
        /* Velocity mode: iq comes from the medium-loop velocity cascade; else manual (D1). */
        cmd.iq_a   = g_mc_inject.velocity_enable ? s_iq_cmd_published : g_mc_inject.iq_cmd_a;
        cmd.enable = true;

        const float vbus = (g_mc_inject.vbus_v > 1.0f) ? g_mc_inject.vbus_v : 24.0f;
        MC_PwmDuty_t duty = MC_Foc_Update(&s_foc, &s_foc_cfg, &cmd, &s_currents, &elec, vbus);

        if (!s_pwm_on) { MC_Pwm_Start(); s_pwm_on = true; }
        MC_Pwm_SetDutyFast(&duty);
        g_mc_debug.pwm_enabled       = true;
        g_mc_debug.id_meas_a         = s_foc.id_measured_a;
        g_mc_debug.iq_meas_a         = s_foc.iq_measured_a;
        g_mc_debug.vd_v              = s_foc.vd_v;
        g_mc_debug.vq_v              = s_foc.vq_v;
        g_mc_debug.voltage_saturated = s_foc.voltage_saturated;
        g_mc_debug.vd_applied_v      = 0.0f;
    }
    else
    {
        s_foc_on = false;

        const float vd = MC_Math_Clamp(g_mc_inject.align_voltage_v, -MC_C2_VD_MAX, MC_C2_VD_MAX);
        if (armed && (vd != 0.0f))
        {
            /* Stage C2: open-loop d-axis voltage at the commanded electrical angle. */
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
    s_elec_angle = s_est.electrical.electrical_angle_rad;   /* publish to fast loop (atomic float) */

    /* Stage D2: velocity cascade -> torque request -> iq, published to the fast loop. */
    const bool vel_active = g_mc_inject.inject_enable && g_mc_inject.foc_enable
                            && g_mc_inject.velocity_enable && !s_oc_trip;
    if (vel_active)
    {
        if (!s_vel_on) { MC_VelocityController_Reset(&s_vel); s_vel_on = true; }

        const float vdem = g_mc_inject.velocity_cmd_rad_s;
        const float vact = s_est.mechanical.velocity_rad_per_s;   /* observer by default */
        const float tcorr = MC_VelocityController_Update(&s_vel, &s_vel_cfg, vdem, vact);

        MC_CurrentRequestDebug_t crd;
        MC_MotorTorqueRequest_t treq =
            MC_CurrentRequest_Update(&s_torque_cfg, tcorr, 0.0f /* accel_ff: D3 */, vact, true, &crd);
        MC_FocCurrentCommand_t fcmd = MC_CurrentRequest_ToFocCommand(&s_torque_cfg, &treq);

        s_iq_cmd_published = fcmd.iq_a;
        g_mc_debug.vel_demand_rad_s  = vdem;
        g_mc_debug.vel_torque_cmd_nm = treq.torque_nm;
        g_mc_debug.vel_iq_cmd_a      = fcmd.iq_a;
    }
    else
    {
        s_vel_on = false;
        s_iq_cmd_published = 0.0f;
        g_mc_debug.vel_iq_cmd_a = 0.0f;
    }

    /* Stage C2: capture the electrical offset so the electrical angle reads 0 at the held
       rotor position (commanded electrical angle 0). */
    if (g_mc_inject.request_align_capture)
    {
        g_mc_inject.request_align_capture = false;
        s_est_cfg.electrical_offset_rad =
            MC_Math_Wrap2Pi(-s_pos_sample.position_rad * s_est_cfg.pole_pairs);
        g_mc_debug.elec_offset_rad = s_est_cfg.electrical_offset_rad;
        calib_save();   /* auto-save: written by the slow loop once the drive is off (ADR-010) */
    }
}

void MC_SlowLoop_10_100Hz(void)
{
    /* Persistence: flash writes only when the power stage is off, to avoid disturbing an
       active drive (ADR-010). The store erases/programs the inactive A/B slot. */
    if (!s_pwm_on)
    {
        if (g_mc_inject.request_factory_reset)
        {
            g_mc_inject.request_factory_reset = false;
            MC_PersistentStore_FactoryReset();
            g_mc_debug.store_valid = false;
        }
        MC_PersistentStore_ServiceSlow();
        g_mc_debug.store_valid = MC_PersistentStore_HasValid();
    }
    g_mc_debug.store_save_pending = MC_PersistentStore_SavePending();
}
