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
#include "mc_od.h"
#include "mc_od_store.h"
#include "mc_comms.h"
#include "mc_mode_manager.h"
#include "mc_if_od.h"      /* MC_IF_*_SCALE, status/mode bits, persistence magics (shared contract) */
#include <math.h>
#include <string.h>

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

/* Mechanical home (multi-turn): the OD position_actual and (later, D3) position commands are
   relative to this captured absolute position. Set via the SET_MECH_ZERO cal command; persisted. */
static float s_home_offset_rad;

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

/* E1: effective drive command (arbitrated commissioning-vs-remote in the medium loop, consumed
   by the fast/medium loops). Plain scalars, single-writer (medium) / reader (fast) — atomic. */
static volatile bool  s_eff_drive;        /* run the closed FOC current loop */
static volatile bool  s_eff_torque_mode;  /* true = direct iq; false = velocity loop */
static volatile float s_eff_iq_cmd;       /* iq command in torque mode [A] */
static volatile float s_eff_id_cmd;       /* id command [A] */
static volatile float s_eff_vel_cmd;      /* velocity demand [rad/s] */
static volatile bool  s_eff_align;        /* commissioning open-loop align active */
static volatile float s_eff_align_v;      /* open-loop d-axis voltage [V] */
static volatile float s_eff_align_angle;  /* open-loop electrical angle [rad] */

/* Gather the full parameter set -- calibration + every persistent OD entry (gains/config) -- and
   latch a flash save (written by the slow loop when the power stage is off). See ADR-010/023. */
static void params_save(void)
{
    MC_Params_t p;
    memset(&p, 0, sizeof p);
    p.calib.electrical_offset_rad      = s_est_cfg.electrical_offset_rad;
    p.calib.current_offset_a_counts    = s_cs.offset_a_counts;
    p.calib.current_offset_c_counts    = s_cs.offset_c_counts;
    p.calib.mechanical_zero_offset_rad = s_enc_cfg.mechanical_zero_offset_rad;
    p.calib.phase_order                = 1;     /* phase-order detection is Phase E */
    p.calib.home_offset_rad            = s_home_offset_rad;
    p.od_blob_len = MC_Od_GatherPersistent(p.od_blob, (uint16_t)sizeof p.od_blob);
    MC_PersistentStore_RequestSave(&p, (uint16_t)sizeof p);
}

/* Apply OD gains (g_od, written via the dictionary) to the live controller configs. Runs in the
   slow loop = a safe update point. Observer gains and the electrical offset stay on the
   watch-window / alignment paths for now (see ADR-015). */
static void od_apply_gains(void)
{
    const float kt   = g_od.motor_kt_nm_per_a;
    const float tlim = g_od.vel_current_limit_a * kt;

    s_vel_cfg.pid.kp = g_od.vel_kp;
    s_vel_cfg.pid.ki = g_od.vel_ki;
    s_vel_cfg.pid.kd = g_od.vel_kd;
    s_vel_cfg.pid.output_min     = -tlim;  s_vel_cfg.pid.output_max     = tlim;
    s_vel_cfg.pid.integrator_min = -tlim;  s_vel_cfg.pid.integrator_max = tlim;
    s_vel_cfg.torque_output_limit_nm = tlim;
    s_torque_cfg.current_limit_a        = g_od.vel_current_limit_a;
    s_torque_cfg.torque_limit_nm        = tlim;
    s_torque_cfg.torque_constant_nm_per_a = kt;

    s_foc_cfg.id_pi.kp = g_od.foc_id_kp;  s_foc_cfg.id_pi.ki = g_od.foc_id_ki;
    s_foc_cfg.iq_pi.kp = g_od.foc_iq_kp;  s_foc_cfg.iq_pi.ki = g_od.foc_iq_ki;
    s_foc_cfg.voltage_limit_v = g_od.foc_voltage_limit_v;

    s_est_cfg.velocity_filter_hz = g_od.est_velocity_filter_hz;
    /* current_trip stays on the watch-window inject path during bring-up (read-reflected in
       od_mirror_live), to avoid a two-writer conflict. */
}

/* Mirror live state into the OD store so reads return current values (telemetry RO; observer
   gains / electrical offset reflect their live source). */
static void od_mirror_live(void)
{
    g_od.tlm_vel_demand_rad_s     = g_mc_debug.vel_demand_rad_s;
    g_od.tlm_vel_actual_rad_s     = g_mc_debug.mech_velocity_rad_s;
    g_od.tlm_vel_iq_cmd_a         = g_mc_debug.vel_iq_cmd_a;
    g_od.tlm_id_meas_a            = g_mc_debug.id_meas_a;
    g_od.tlm_iq_meas_a            = g_mc_debug.iq_meas_a;
    g_od.tlm_vd_v                 = g_mc_debug.vd_v;
    g_od.tlm_vq_v                 = g_mc_debug.vq_v;
    g_od.tlm_electrical_angle_rad = g_mc_debug.elec_angle_rad;
    g_od.tlm_mech_position_rad    = g_mc_debug.mech_position_rad;
    g_od.tlm_mech_velocity_rad_s  = g_mc_debug.mech_velocity_rad_s;
    g_od.tlm_bus_voltage_v        = g_mc_inject.vbus_v;   /* no Vbus sensor yet */

    g_od.est_electrical_offset_rad = s_est_cfg.electrical_offset_rad;
    g_od.est_obs_kp = g_mc_inject.obs_kp;
    g_od.est_obs_ki = g_mc_inject.obs_ki;
    g_od.est_obs_kv = g_mc_inject.obs_kv;
    g_od.est_use_observer = g_mc_inject.use_finite_diff_velocity ? 0u : 1u;
    g_od.current_trip_a   = g_mc_inject.current_limit_a;

    /* CiA-402 standard objects (REQ-0001): RO actuals mirrored (scaled to wire units),
       status/error derived. RW objects (controlword/modes/targets) are stored and consumed
       by the mode manager (E1); full state-machine behaviour lands there. */
    g_od.position_actual = (int32_t)((g_mc_debug.mech_position_rad - s_home_offset_rad) / MC_IF_POS_SCALE);
    g_od.velocity_actual = (int32_t)(g_mc_debug.mech_velocity_rad_s / MC_IF_VEL_SCALE);
    g_od.torque_actual   = (int32_t)(g_mc_debug.iq_meas_a           / MC_IF_CUR_SCALE);
    /* statusword + modes_of_operation_display are owned by the E1 arbiter (above). */
    g_od.error_code      = 0u;
    g_od.error_register  = 0u;
    g_od.fault_flags     = 0u;
    g_od.motor_resistance_ohm = s_motor.resistance_ohm;
    g_od.motor_inductance_h   = s_motor.inductance_h;
    g_od.store_status    = (uint16_t)(MC_PersistentStore_HasValid() ? 1u : 0u);
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

    /* Object dictionary: seed defaults (its gains match the configs seeded above). */
    MC_Od_Init();
    MC_Comms_Init();        /* SPI protocol handler (transport DMA wired in F2b) */
    MC_ModeManager_Init();  /* CiA-402 drive state machine (E1) */

    /* Load persisted calibration (electrical offset + current offsets) if present. */
    if (MC_PersistentStore_Init() == MC_OK)
    {
        MC_Params_t p;
        if (MC_PersistentStore_Read(&p, (uint16_t)sizeof p) == MC_OK)
        {
            s_est_cfg.electrical_offset_rad      = p.calib.electrical_offset_rad;
            s_enc_cfg.mechanical_zero_offset_rad = p.calib.mechanical_zero_offset_rad;
            s_cs.offset_a_counts                 = p.calib.current_offset_a_counts;
            s_cs.offset_c_counts                 = p.calib.current_offset_c_counts;
            s_cs.calibrated                      = true;
            s_home_offset_rad                    = p.calib.home_offset_rad;
            MC_Od_RestorePersistent(p.od_blob, p.od_blob_len);  /* gains/config back into g_od */
            g_mc_debug.elec_offset_rad           = p.calib.electrical_offset_rad;
            g_mc_debug.home_offset_rad           = p.calib.home_offset_rad;
            g_mc_debug.store_valid               = true;
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

    const bool blocked = s_oc_trip || g_mc_inject.request_offset_cal;

    if (!blocked && s_eff_drive)
    {
        /* Closed FOC current loop (commissioning or remote -- effective command, E1). */
        if (!s_foc_on) { MC_Foc_Reset(&s_foc); s_foc_on = true; }

        MC_ElectricalState_t elec;
        elec.electrical_angle_rad          = s_elec_angle;   /* published by the medium loop */
        elec.electrical_velocity_rad_per_s = 0.0f;
        elec.electrical_valid              = true;

        MC_FocCurrentCommand_t cmd;
        cmd.id_a   = s_eff_id_cmd;
        /* Torque mode: direct iq. Velocity mode: iq from the medium-loop velocity cascade. */
        cmd.iq_a   = s_eff_torque_mode ? s_eff_iq_cmd : s_iq_cmd_published;
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

        const float vd = MC_Math_Clamp(s_eff_align_v, -MC_C2_VD_MAX, MC_C2_VD_MAX);
        if (!blocked && s_eff_align && (vd != 0.0f))
        {
            /* Commissioning open-loop d-axis voltage at the commanded electrical angle (C2). */
            float sin_e, cos_e;
            MC_Math_SinCos(s_eff_align_angle, &sin_e, &cos_e);
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

    /* E1: arbitrate the command source -- commissioning (watch window) vs remote (OD/CiA-402 via
       the mode manager) -- into the effective command the loops consume. See ADR-018. */
    {
        MC_DriveCommand_t dc;
        dc.controlword               = g_od.controlword;
        dc.mode_of_operation         = g_od.modes_of_operation;
        dc.target_position_rad       = (float)g_od.target_position * MC_IF_POS_SCALE;
        dc.target_velocity_rad_per_s = (float)g_od.target_velocity * MC_IF_VEL_SCALE;
        dc.target_torque_nm          = 0.0f;
        dc.requested_time_s          = 0.0f;
        dc.new_setpoint              = (g_od.controlword & MC_IF_CW_NEW_SETPOINT) != 0u;
        dc.halt                      = false;
        dc.fault_reset               = (g_od.controlword & MC_IF_CW_FAULT_RESET) != 0u;

        MC_FaultState_t fs = {0};
        fs.severe_active = s_oc_trip;
        MC_ModeManager_Update(&dc, &fs);
        const MC_DriveStatus_t ds = MC_ModeManager_GetStatus();

        if (g_mc_inject.inject_enable)
        {
            /* Commissioning: identical to the watch-window behaviour. */
            s_eff_align       = (!g_mc_inject.foc_enable) && (g_mc_inject.align_voltage_v != 0.0f);
            s_eff_align_v     = g_mc_inject.align_voltage_v;
            s_eff_align_angle = g_mc_inject.align_angle_rad;
            s_eff_drive       = g_mc_inject.foc_enable;
            s_eff_torque_mode = !g_mc_inject.velocity_enable;
            s_eff_iq_cmd      = g_mc_inject.iq_cmd_a;
            s_eff_id_cmd      = g_mc_inject.id_cmd_a;
            s_eff_vel_cmd     = g_mc_inject.velocity_cmd_rad_s;
            g_od.statusword   = (uint16_t)((g_mc_debug.pwm_enabled ? MC_IF_SW_ENABLED : 0u)
                                         | (s_oc_trip ? MC_IF_SW_FAULT : 0u) | MC_IF_SW_READY);
        }
        else
        {
            /* Remote: the mode manager (OD/CiA-402) drives. Boot-safe (controlword 0 = Disabled). */
            s_eff_align = false;
            s_eff_drive = ds.operation_enabled;
            if (ds.active_mode == MC_MODE_TORQUE_CURRENT)
            {
                s_eff_torque_mode = true;
                s_eff_iq_cmd      = (float)g_od.target_torque * MC_IF_CUR_SCALE;
                s_eff_id_cmd      = 0.0f;
            }
            else if (ds.active_mode == MC_MODE_PROFILE_VELOCITY)
            {
                s_eff_torque_mode = false;
                s_eff_vel_cmd     = dc.target_velocity_rad_per_s;   /* = cyclic velocity_setpoint (v3) */
            }
            else
            {
                s_eff_drive = false;   /* disabled / quick-stop / position (not routed yet) -> safe */
            }
            if (dc.fault_reset) { g_mc_inject.clear_fault = true; }   /* clear oc_trip in the fast loop */
            g_od.statusword    = ds.statusword;
            g_od.modes_of_operation_display = g_od.modes_of_operation;
        }
    }

    /* Stage D2: velocity cascade -> torque request -> iq, published to the fast loop. */
    const bool vel_active = s_eff_drive && !s_eff_torque_mode && !s_oc_trip;
    if (vel_active)
    {
        if (!s_vel_on) { MC_VelocityController_Reset(&s_vel); s_vel_on = true; }

        const float vdem = s_eff_vel_cmd;
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
        params_save();   /* auto-save: written by the slow loop once the drive is off (ADR-010) */
    }

    /* Set mechanical zero (home): capture the current absolute (multi-turn) position as the home
       reference, so OD position_actual + (D3) position commands are relative to it. Auto-saved. */
    if (g_mc_inject.request_set_mech_zero)
    {
        g_mc_inject.request_set_mech_zero = false;
        s_home_offset_rad          = s_est.mechanical.position_rad;
        g_mc_debug.home_offset_rad = s_home_offset_rad;
        params_save();
    }

    od_mirror_live();   /* publish live state into the OD store */
}

void MC_SlowLoop_10_100Hz(void)
{
    /* OD-triggered persistence commands (0x2800), via the shared magics. */
    if (g_od.store_factory_reset == MC_IF_FACTORY_RESET_MAGIC)
    {
        g_mc_inject.request_factory_reset = true;
        g_od.store_factory_reset = 0u;
    }
    if (g_od.store_save_command == MC_IF_SAVE_MAGIC)
    {
        params_save();
        g_od.store_save_command = 0u;
    }

    /* OD calibration command (0x2700:1). SET_MECH_ZERO captures the current position as the
       mechanical home -- the capture runs in the medium loop; persistence follows when drive off. */
    if (g_od.cal_command == MC_IF_CAL_SET_MECH_ZERO)
    {
        g_mc_inject.request_set_mech_zero = true;
        g_od.cal_status  = MC_IF_CAL_SET_MECH_ZERO;   /* accepted; echoes the last command */
        g_od.cal_command = MC_IF_CAL_NONE;
    }

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

    od_apply_gains();   /* apply OD-written gains to the live controllers (safe update point) */

    /* Inter-MCU command dead-man (REMOTE mode only): a stale cyclic-command stream zeroes the
       remote velocity demand (the OD target). Skipped in commissioning so the watch-window
       command is never clobbered. Full quick-stop is the fault manager's job (E2). */
    if (!g_mc_inject.inject_enable && MC_Comms_CommandTimedOut())
    {
        g_od.target_velocity = 0;
    }
}
