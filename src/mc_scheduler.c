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
#include "mc_pid.h"        /* brushed armature-current PI (ADR-039) */
#include "mc_hbridge.h"    /* brushed-DC locked anti-phase modulator (ADR-039) */
#include "mc_dac.h"        /* debug DAC output on PA4 for scoping (ADR-005) */
#include "mc_persistent_store.h"
#include "mc_calib_data.h"
#include "mc_velocity_controller.h"
#include "mc_current_request.h"
#include "mc_trajectory.h"
#include "mc_position_controller.h"
#include "mc_signal_gen.h"
#include "mc_od.h"
#include "mc_od_store.h"
#include "mc_comms.h"
#include "mc_mode_manager.h"
#include "mc_if_od.h"      /* MC_IF_*_SCALE, status/mode bits, persistence magics (shared contract) */
#include "mc_if_protocol.h" /* MC_IF_MOVE_* cyclic-header movement_status bits (REQ-0013/ADR-033) */
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
static bool  s_pos_locked;   /* false until the drive is first enabled; while false the startup anchor
                                re-derives continuous from the absolute encoder each cycle (ADR-037/038) */

/* Stage C2: open-loop drive state. */
#define MC_C2_VD_MAX 3.0f   /* hard clamp on commanded d-axis voltage [V] */
static bool s_oc_trip;      /* latched over-current trip */
static bool s_pwm_on;       /* PWM outputs currently enabled */

/* Electrical-alignment routine (ADR-024): current-regulated open-loop drive at electrical angle 0. */
typedef enum { MC_ALIGN_IDLE = 0, MC_ALIGN_RUN } MC_AlignState_t;
static MC_AlignState_t s_align_state;
static uint32_t        s_align_ticks_left;   /* medium-loop (1 ms) ticks left in the drive/hold */
static float           s_align_vd;           /* regulated open-loop d-axis voltage [V] */
#define MC_ALIGN_KI_V_PER_A (0.003f)         /* slow Vd current-regulator gain [V per A-err per tick] */
#define MC_CAL_STATUS_FAULT (0xFFFFu)        /* cal_status (0x2700:2) value on a faulted calibration */

/* Stage D1: FOC current loop. */
static MC_Foc_t       s_foc;
static MC_FocConfig_t s_foc_cfg;
static bool           s_foc_on;       /* FOC active (for entry reset) */
static volatile float s_elec_angle;   /* electrical angle published medium->fast (atomic float) */

/* Brushed-DC backend (ADR-039): single armature-current PI -> locked anti-phase H-bridge voltage. */
static MC_Pid_t       s_hb_ipi;
static MC_PidConfig_t s_hb_ipi_cfg;

/* Stage D2: velocity loop + torque/current request (runs in the medium loop). */
static MC_MotorModel_t               s_motor;
static MC_VelocityController_t       s_vel;
static MC_VelocityControllerConfig_t s_vel_cfg;
static MC_TorqueModelConfig_t        s_torque_cfg;
static volatile float                s_iq_cmd_published;  /* velocity-loop iq, medium->fast (atomic) */
static bool                          s_vel_on;            /* velocity loop active (for entry reset) */

/* Stage D3: trajectory + position loop (runs in the medium loop; feeds the velocity cascade). */
static MC_TrajectoryPlanner_t        s_traj;
static MC_PositionController_t       s_pos_ctl;
static MC_PositionControllerConfig_t s_pos_cfg;
static bool                          s_eff_position_mode; /* PROFILE_POSITION active (medium-loop arbiter) */
static bool                          s_pos_on;            /* position loop active (for entry reset) */
static float                         s_accel_ff_rad_s2;   /* trajectory accel feedforward -> torque request */
static float                         s_vel_ff_gain;       /* velocity FF ratio in the position cascade (0x2200:4, ADR-031) */
static float                         s_pos_hold_rad;      /* held position when in position mode with no active plan */

/* Loop-tuning test-signal overlay (ADR-030): an on-motor generator drives the selected loop's reference. */
static MC_SignalGen_t                s_sig_gen;
static uint8_t                       s_sig_loop;           /* latched target loop while active (MC_IF_TEST_MODE_*) */
static float                         s_sig_value;          /* generator output this medium tick */
static float                         s_pos_tune_entry_rad; /* position captured when position-tuning fires (home-relative) */
#define MC_POS_TARGET_WINDOW_RAD (0.01f)                  /* |error| under this + trajectory complete -> target reached */

/* E1: effective drive command (arbitrated commissioning-vs-remote in the medium loop, consumed
   by the fast/medium loops). Plain scalars, single-writer (medium) / reader (fast) — atomic. */
static volatile bool  s_eff_drive;        /* run the closed FOC current loop */
static volatile bool  s_eff_halt;         /* HALT: hold current position, stay enabled (ADR-035) */
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
    /* Current offsets: persist the measured values only if a current-offset calibration has run;
       otherwise write a 0 "not measured" sentinel so a restore can't report current-offset
       calibration as done when it merely reloaded board-nominal offsets (ADR-026). */
    if (s_cs.calibrated)
    {
        p.calib.current_offset_a_counts = s_cs.offset_a_counts;
        p.calib.current_offset_c_counts = s_cs.offset_c_counts;
    }
    else
    {
        p.calib.current_offset_a_counts = 0.0f;
        p.calib.current_offset_c_counts = 0.0f;
    }
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

    {
        /* vel_load_factor (0x2300:5, REQ-0014/ADR-034): operator load multiplier on the velocity-loop
           gains, clamped to [0.3, 2.0] so a stray/zero write can't kill or blow up the loop. */
        const float lf = (g_od.vel_load_factor < 0.3f) ? 0.3f
                       : (g_od.vel_load_factor > 2.0f) ? 2.0f : g_od.vel_load_factor;
        s_vel_cfg.pid.kp = g_od.vel_kp * lf;
        s_vel_cfg.pid.ki = g_od.vel_ki * lf;
    }
    s_vel_cfg.pid.kd = g_od.vel_kd;
    s_vel_cfg.pid.output_min     = -tlim;  s_vel_cfg.pid.output_max     = tlim;
    s_vel_cfg.pid.integrator_min = -tlim;  s_vel_cfg.pid.integrator_max = tlim;
    s_vel_cfg.torque_output_limit_nm = tlim;
    s_torque_cfg.current_limit_a        = g_od.vel_current_limit_a;
    s_torque_cfg.torque_limit_nm        = tlim;
    s_torque_cfg.torque_constant_nm_per_a = kt;

    /* Over-current trip threshold (measured |phase current|, fast loop): driven by the OD entry
       current_trip_a (0x2600:2) -- GUI-settable + PERSIST. Clamp to a small positive minimum so a
       stray 0 / negative can't latch the trip permanently and lock the drive out. (ADR-029) */
    g_mc_inject.current_limit_a = (g_od.current_trip_a > 0.1f) ? g_od.current_trip_a : 0.1f;

    /* Position loop (D3, ADR-028): P-default gains (0x2200); velocity correction capped at the
       profile velocity (0x6081), falling back to 10 rad/s if unset. */
    s_pos_cfg.pid.kp = g_od.pos_kp;
    s_pos_cfg.pid.ki = g_od.pos_ki;
    s_pos_cfg.pid.kd = g_od.pos_kd;
    s_vel_ff_gain    = (g_od.velocity_ff_gain >= 0.0f) ? g_od.velocity_ff_gain : 0.0f;  /* 0x2200:4 (ADR-031) */
    {
        const float vlim = (float)g_od.profile_velocity * MC_IF_VEL_SCALE;
        s_pos_cfg.velocity_correction_limit_rad_per_s = (vlim > 0.1f) ? vlim : 10.0f;
        s_pos_cfg.pid.output_min = -s_pos_cfg.velocity_correction_limit_rad_per_s;
        s_pos_cfg.pid.output_max =  s_pos_cfg.velocity_correction_limit_rad_per_s;
    }

    s_foc_cfg.id_pi.kp = g_od.foc_id_kp;  s_foc_cfg.id_pi.ki = g_od.foc_id_ki;
    s_foc_cfg.iq_pi.kp = g_od.foc_iq_kp;  s_foc_cfg.iq_pi.ki = g_od.foc_iq_ki;
    s_foc_cfg.voltage_limit_v = g_od.foc_voltage_limit_v;
    /* Brushed current loop: R/L are config (0x2000:3,4 -> the model); the gains are DERIVED from R/L +
       bandwidth (0x2400:8) and reported read-only at 0x2400:6,7. kp = wc*L, ki = wc*R cancels the winding pole. */
    s_motor.resistance_ohm = g_od.motor_resistance_ohm;
    s_motor.inductance_h   = g_od.motor_inductance_h;
    {
        const float wc  = (g_od.hb_cur_bandwidth > 1.0f) ? g_od.hb_cur_bandwidth : 1500.0f;
        s_hb_ipi_cfg.kp = wc * g_od.motor_inductance_h;
        s_hb_ipi_cfg.ki = wc * g_od.motor_resistance_ohm;
        g_od.hb_cur_kp  = s_hb_ipi_cfg.kp;   /* RO readback of the derived gains */
        g_od.hb_cur_ki  = s_hb_ipi_cfg.ki;
    }

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
    g_od.tlm_i_arm_a              = g_mc_debug.i_arm_a;   /* brushed armature current (0x2410:6) */
    g_od.tlm_mech_position_rad    = g_mc_debug.mech_position_rad;
    g_od.tlm_mech_velocity_rad_s  = g_mc_debug.mech_velocity_rad_s;
    g_od.tlm_pos_demand_rad       = g_mc_debug.pos_demand_rad;   /* abs position demand (0x2510:3 PDO) -- graph vs 0x6064 */
    g_od.tlm_bus_voltage_v        = g_mc_inject.vbus_v;   /* no Vbus sensor yet */

    g_od.est_electrical_offset_rad = s_est_cfg.electrical_offset_rad;   /* cal result -- display only (RO) */
    /* est_obs_kp/ki/kv + est_use_observer (0x2500:3-6) are OD config applied in the medium loop --
       NOT mirrored here, so GUI writes stick (audit fix; were overwritten from g_mc_inject). */
    /* current_trip_a (0x2600:2) is OD-sourced now -- applied to the live trip in od_apply_gains.
       Do NOT mirror the live value back here: it would clobber a GUI/OD write every cycle (the
       original "can't set the trip from the GUI" bug). (ADR-029) */

    /* CiA-402 standard objects (REQ-0001): RO actuals mirrored (scaled to wire units),
       status/error derived. RW objects (controlword/modes/targets) are stored and consumed
       by the mode manager (E1); full state-machine behaviour lands there. */
    g_od.position_actual = (int32_t)((g_mc_debug.mech_position_rad - s_home_offset_rad) / MC_IF_POS_SCALE);
    g_od.velocity_actual = (int32_t)(g_mc_debug.mech_velocity_rad_s / MC_IF_VEL_SCALE);
    g_od.torque_actual   = (int32_t)(g_mc_debug.iq_meas_a           / MC_IF_CUR_SCALE);

    /* movement_status (REQ-0013/ADR-033) -> pushed to the fixed cyclic header. MOVING = enabled and the
       axis is commanded or measured to be turning; ON_TARGET = position-loop target reached;
       AT_LIMIT_LO/HI = at/past a manually-set soft position limit (ADR-040). */
    {
        const float vdem = s_eff_vel_cmd;
        const float vact = g_mc_debug.mech_velocity_rad_s;
        uint16_t    ms   = 0u;
        if (s_eff_drive && ((vdem > 0.01f) || (vdem < -0.01f) || (vact > 0.01f) || (vact < -0.01f)))
        {
            ms |= MC_IF_MOVE_MOVING;
        }
        if (g_mc_debug.target_reached) { ms |= MC_IF_MOVE_ON_TARGET; }
        /* Soft position limits (ADR-040): flag AT_LIMIT_LO/HI when at/past a manually-set limit. lo>=hi = off. */
        if (g_od.pos_limit_hi_rad > g_od.pos_limit_lo_rad)
        {
            const float pos_rel = s_est.mechanical.position_rad - s_home_offset_rad;
            if (pos_rel <= g_od.pos_limit_lo_rad) { ms |= MC_IF_MOVE_AT_LIMIT_LO; }
            if (pos_rel >= g_od.pos_limit_hi_rad) { ms |= MC_IF_MOVE_AT_LIMIT_HI; }
        }
        g_mc_debug.movement_status = ms;        /* watch-window mirror */
        MC_Comms_SetMovementStatus(ms);
    }
    /* statusword + modes_of_operation_display are owned by the E1 arbiter (above). */
    g_od.error_code      = 0u;
    g_od.error_register  = 0u;
    g_od.fault_flags     = 0u;
    /* motor_resistance/inductance (0x2000:3,4) are now config inputs (applied in od_apply_gains),
       no longer mirrored from the model here -- writing them sticks (ADR-039 R/L promotion). */
    g_od.store_status    = (uint16_t)((MC_PersistentStore_HasValid()   ? MC_IF_STORE_VALID   : 0u)
                                    | (MC_PersistentStore_SavePending() ? MC_IF_STORE_PENDING : 0u));

    /* Calibration completeness (0x2700:5, ADR-026): derived from existing state. A set bit means that
       calibration currently has valid data; a clear bit means it is still outstanding. */
    {
        uint16_t done = 0u;
        if (s_est_cfg.electrical_offset_rad != 0.0f) { done |= MC_IF_CAL_DONE_ELECTRICAL; }
        if (s_home_offset_rad != 0.0f)               { done |= MC_IF_CAL_DONE_MECH_ZERO; }
        if (s_cs.calibrated)                         { done |= MC_IF_CAL_DONE_CURRENT_OFFSET; }
        g_od.cal_done_flags = done;
    }
}

void MC_Framework_Init(void)
{
    MC_Debug_Init();
    g_mc_debug.fw_build = 51u;   /* build/version marker (ADR-038/039/040): read in the watch window to confirm the flashed image */
    MC_CurrentSense_Init(&s_cs);
    MC_Dac_Init();                         /* start DAC1_OUT1 (PA4) for the debug current scope output */

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
    /* Observer gains + velocity-source default live in the OD (0x2500:3-6, seeded in mc_od.c). */

    /* C2 drive defaults (drive stays off until inject_enable is set). */
    g_mc_inject.vbus_v          = 24.0f;   /* set to your actual supply voltage */
    g_mc_inject.current_limit_a = 3.0f;    /* over-current trip [A] (headroom over ~1.5-2 A breakaway) */
    g_mc_inject.dac_scale_v_per_a = 1.0f;  /* debug DAC (PA4): 1 A -> 1 V (0..3.3 A full scale) */

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

    /* Brushed-DC armature-current PI (ADR-039): runs at the fast-loop rate like FOC; its output is the
       motor voltage command fed to the locked anti-phase modulator. Gains start gentle and are live-
       tunable from the watch window (hb_kp/hb_ki) -- retune for the brushed motor's R/L during bring-up. */
    {
        MC_Pid_Init(&s_hb_ipi);
        MC_Pid_SetEnabled(&s_hb_ipi, true);
        s_hb_ipi_cfg.kp                        = 5.55f;   /* bootstrap; od_apply_gains overwrites from 0x2400:6 hb_cur_kp */
        s_hb_ipi_cfg.ki                        = 6300.0f; /* bootstrap; od_apply_gains overwrites from 0x2400:7 hb_cur_ki */
        s_hb_ipi_cfg.kd                        = 0.0f;
        s_hb_ipi_cfg.sample_period_s           = MC_FAST_DT_S;
        s_hb_ipi_cfg.output_min                = -24.0f;   /* |v| <= bus; the modulator clamps the duty by max_modulation */
        s_hb_ipi_cfg.output_max                =  24.0f;
        s_hb_ipi_cfg.integrator_min            = -24.0f;
        s_hb_ipi_cfg.integrator_max            =  24.0f;
        s_hb_ipi_cfg.derivative_filter_hz      = 0.0f;
        s_hb_ipi_cfg.integrator_enabled        = true;
        s_hb_ipi_cfg.derivative_enabled        = false;
        s_hb_ipi_cfg.derivative_on_measurement = true;
    }

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

    /* Position loop + trajectory planner (D3, ADR-028). P-default; gains refreshed from the OD by
       od_apply_gains each slow tick. */
    {
        MC_PidConfig_t ppid;
        memset(&ppid, 0, sizeof ppid);
        ppid.kp                        = g_od.pos_kp;
        ppid.ki                        = g_od.pos_ki;
        ppid.kd                        = g_od.pos_kd;
        ppid.sample_period_s           = MC_MOTION_DT_S;
        ppid.integrator_enabled        = false;   /* P-default */
        ppid.derivative_enabled        = false;
        ppid.derivative_on_measurement = true;
        s_pos_cfg.pid                                 = ppid;
        s_pos_cfg.velocity_correction_limit_rad_per_s = 10.0f;    /* refreshed from profile_velocity */
        s_pos_cfg.following_error_limit_rad           = 6.2832f;  /* ~1 rev error guard */
    }
    MC_PositionController_Init(&s_pos_ctl);
    MC_Trajectory_Init(&s_traj);
    MC_SignalGen_Init(&s_sig_gen);

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
            /* Current offsets: a saved 0 is the "not measured" sentinel (ADR-026) -- keep the nominal
               init offsets and leave calibrated=false so completeness reports it outstanding. */
            if ((p.calib.current_offset_a_counts != 0.0f) || (p.calib.current_offset_c_counts != 0.0f))
            {
                s_cs.offset_a_counts             = p.calib.current_offset_a_counts;
                s_cs.offset_c_counts             = p.calib.current_offset_c_counts;
                s_cs.calibrated                  = true;
            }
            s_home_offset_rad                    = p.calib.home_offset_rad;
            MC_Od_RestorePersistent(p.od_blob, p.od_blob_len);  /* gains/config back into g_od */
            g_mc_debug.elec_offset_rad           = p.calib.electrical_offset_rad;
            g_mc_debug.home_offset_rad           = p.calib.home_offset_rad;
            g_mc_debug.store_valid               = true;
        }
    }

    /* Apply the selected drive backend (0x2000:6 motor_backend_sel, persisted; default 0 = BLDC/FOC).
       Per-board, so it is read once here at boot: it picks the dispatch path and the current-sense ADC
       channel (FOC stays on ADC2_IN6; brushed repoints ADC2 to IN7 = the new board's I_A). See ADR-039. */
    if (g_od.motor_backend_sel == 1u)
    {
        s_motor.backend_type = MC_MOTOR_BACKEND_BRUSHED_DC_HBRIDGE;
        MC_CurrentSense_SelectHBridgeLegs();
    }
    g_mc_debug.backend_type = (uint8_t)s_motor.backend_type;

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

    /* Drive-backend selection (ADR-039) -- evaluated before the over-current monitor because the brushed
       H-bridge senses only the two driven legs: its trip uses the armature leg alone. The 3-phase
       reconstruction ib = -(ia+ic) is invalid when the third leg is disconnected -- its shunt input
       floats and rails, which false-trips at ~2x the rail current (~65 A seen on the new board, where the
       old I_C input PC0/ADC2_IN6 is unused). The backend is selected by brushed_backend
       (watch window) regardless of inject_enable -- so a remote/PC current command drives the brushed
       loop too -- or by s_motor.backend_type for a fixed brushed build. See ADR-005/039. */
    const bool brushed = g_mc_inject.brushed_backend
                         || (s_motor.backend_type == MC_MOTOR_BACKEND_BRUSHED_DC_HBRIDGE);
    g_mc_debug.backend_type = brushed ? 1u : 0u;

    /* Stage C2: over-current monitor. FOC: max over all three phases. Brushed: the armature leg on ADC1
       only (s_currents.ia_a); ib/ic involve the disconnected leg and would false-trip. */
    float imax = fabsf(s_currents.ia_a);
    if (!brushed)
    {
        const float aib = fabsf(s_currents.ib_a);
        const float aic = fabsf(s_currents.ic_a);
        if (aib > imax) { imax = aib; }
        if (aic > imax) { imax = aic; }
    }
    g_mc_debug.i_max_a = imax;

    if (g_mc_inject.clear_fault) { s_oc_trip = false; g_mc_inject.clear_fault = false; }
    if (s_currents.valid && (imax > g_mc_inject.current_limit_a)) { s_oc_trip = true; }

    const bool blocked = s_oc_trip || g_mc_inject.request_offset_cal;

    if (brushed && !blocked && s_eff_drive)
    {
        /* Armature current for the loop. The new board's ADC1 reads leg B (I_B = -I_A), so s_currents.ia_a
           is the NEGATIVE of the forward armature current -- negate it so the feedback sign matches the
           command. (An inverted measurement is positive feedback: the integrator runs the current away.
           Magnitude was verified vs a meter; the DAC shows |i| and was correct, only the sign was wrong.)
           Dual-leg (I_A - I_B)/2 lands once ADC2 reads IN7 = I_A. */
        const float i_arm = -s_currents.ia_a;
        const float i_cmd = s_eff_torque_mode ? s_eff_iq_cmd : s_iq_cmd_published;

        /* Open-loop voltage (bring-up: verify current sign/scaling) OR the closed armature-current PI.
           Open-loop holds the PI reset so closing it afterwards is bumpless. Gains are live-tunable. */
        float v_cmd;
        if (g_mc_inject.hb_open_loop)
        {
            v_cmd = g_mc_inject.hb_voltage_v;
            MC_Pid_Reset(&s_hb_ipi);
        }
        else
        {
            v_cmd = MC_Pid_Update(&s_hb_ipi, &s_hb_ipi_cfg, i_cmd, i_arm);   /* gains from od_apply_gains (0x2400:6,7) */
        }

        const float vbus = (g_mc_inject.vbus_v > 1.0f) ? g_mc_inject.vbus_v : 24.0f;
        MC_HBridgeConfig_t hbcfg;
        hbcfg.vbus_v         = vbus;
        hbcfg.max_modulation = 0.95f;
        MC_PwmDuty_t duty = MC_HBridge_LockedAntiphase(v_cmd, &hbcfg);

        if (!s_pwm_on) { MC_Pwm_Start(); s_pwm_on = true; }
        MC_Pwm_SetDutyFast(&duty);
        g_mc_debug.pwm_enabled  = true;
        g_mc_debug.i_arm_a      = i_arm;
        g_mc_debug.v_cmd_v      = v_cmd;
        g_mc_debug.vd_applied_v = 0.0f;
        s_foc_on = false;
    }
    else if (brushed)
    {
        /* Brushed but disabled/blocked -> safe-off and hold the current PI at zero (anti-windup). */
        MC_Pid_Reset(&s_hb_ipi);
        if (s_pwm_on) { MC_Pwm_ForceSafeOff(); s_pwm_on = false; }
        g_mc_debug.pwm_enabled  = false;
        g_mc_debug.i_arm_a      = -s_currents.ia_a;   /* forward-positive (ADC1 = I_B = -I_A on this board) */
        g_mc_debug.v_cmd_v      = 0.0f;
        g_mc_debug.vd_applied_v = 0.0f;
        s_foc_on = false;
    }
    else if (!blocked && s_eff_drive)
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

    /* Debug DAC (PA4 / DAC1_OUT1): mirror i_max_a to the scope, scaled by dac_scale_v_per_a (default
       1 V/A -> 1 A = 1 V), clamped to 0..Vref. Output saturates at ~3.3 A with the default scale. */
    MC_Dac_SetVolts(g_mc_debug.i_max_a * g_mc_inject.dac_scale_v_per_a);
}

void MC_MotionLoop_1kHz(void)
{
    /* Stage B2: read the SSI encoder and update the state estimator. */
    /* Apply observer tuning + velocity-source selection from the OD (0x2500:3-6, GUI-settable + PERSIST). */
    s_est_cfg.obs_kp       = g_od.est_obs_kp;
    s_est_cfg.obs_ki       = g_od.est_obs_ki;
    s_est_cfg.obs_kv       = g_od.est_obs_kv;
    s_est_cfg.use_observer = (g_od.est_use_observer != 0u);

    if (MC_SsiEncoder_ReadHardware(&s_enc, &s_enc_cfg, &s_pos_sample))
    {
        MC_StateEstimator_Update(&s_est, &s_est_cfg, &s_pos_sample);

        /* Startup position anchor (ADR-037, hardened by ADR-038). The single-turn absolute encoder
           loses the turn count across a power cycle, so the continuous position must be anchored to the
           home-relative reading wrapped to the nearest turn. The original one-shot seed (s_pos_seeded)
           raced the persistent home load on a COLD boot: it could fire with s_home_offset_rad still 0,
           leaving continuous ~1 turn off everywhere except home (a soft reset hid it -- RAM kept the
           good anchor so the seed never re-ran). Hardened: while the drive has NEVER been enabled,
           re-anchor every cycle -- idempotent once correct, and self-correcting if home loads late or
           the encoder is slow to read. s_pos_locked latches on the first enable so motion thereafter
           tracks true multi-turn (deltas accumulate past +/-pi without being wrapped back). */
        if (s_eff_drive)
        {
            s_pos_locked = true;   /* drive engaged -> freeze the anchor; track multi-turn from here */
        }
        else if (!s_pos_locked && s_pos_sample.valid)
        {
            float home_rel = s_pos_sample.position_rad - s_home_offset_rad;
            while (home_rel >  3.14159265358979324f) { home_rel -= 6.28318530717958648f; }
            while (home_rel < -3.14159265358979324f) { home_rel += 6.28318530717958648f; }
            MC_StateEstimator_SeedContinuous(&s_est, s_home_offset_rad + home_rel);
        }
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
            s_eff_position_mode = false;   /* commissioning never uses the position cascade */
            s_eff_halt          = false;
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
            const bool halt_rise = (ds.active_mode == MC_MODE_POSITION_HOLD) && !s_eff_halt;
            s_eff_halt = (ds.active_mode == MC_MODE_POSITION_HOLD);
            if (ds.active_mode == MC_MODE_TORQUE_CURRENT)
            {
                s_eff_torque_mode   = true;
                s_eff_position_mode = false;
                s_eff_iq_cmd        = (float)g_od.target_torque * MC_IF_CUR_SCALE;
                s_eff_id_cmd        = 0.0f;
            }
            else if (ds.active_mode == MC_MODE_PROFILE_VELOCITY)
            {
                s_eff_torque_mode   = false;
                s_eff_position_mode = false;
                s_eff_vel_cmd       = dc.target_velocity_rad_per_s;   /* = cyclic velocity_setpoint (v3) */
            }
            else if (ds.active_mode == MC_MODE_PROFILE_POSITION)
            {
                /* D3 (ADR-028): the position cascade below produces s_eff_vel_cmd; run it as a velocity
                   move. NEW_SETPOINT (rising edge, latched by the mode manager) starts a fresh plan. */
                s_eff_torque_mode   = false;
                s_eff_position_mode = true;
                if (ds.new_setpoint_latched)
                {
                    MC_TrajRequest_t req;
                    req.start.position_rad             = s_est.mechanical.position_rad - s_home_offset_rad;
                    req.start.velocity_rad_per_s       = 0.0f;
                    req.start.acceleration_rad_per_s2  = 0.0f;
                    {
                        float tgt = (float)g_od.target_position * MC_IF_POS_SCALE;
                        const float lo = g_od.pos_limit_lo_rad, hi = g_od.pos_limit_hi_rad;
                        if (hi > lo) { if (tgt > hi) { tgt = hi; } else if (tgt < lo) { tgt = lo; } }  /* soft limits (ADR-040) */
                        req.target_position_rad        = tgt;
                    }
                    req.target_velocity_rad_per_s      = 0.0f;
                    req.target_acceleration_rad_per_s2 = 0.0f;
                    req.requested_time_s               = (float)g_od.target_position_time_ms * 0.001f;
                    {
                        float vmax = (float)g_od.profile_velocity     * MC_IF_VEL_SCALE;
                        float amax = (float)g_od.profile_acceleration * MC_IF_ACC_SCALE;
                        float dmax = (float)g_od.profile_deceleration * MC_IF_ACC_SCALE;
                        /* Motor safety envelope (ADR-040): clamp the CMC's requested profile to the
                           motor-owned ceiling (0 = disabled). The motor is the authority here. */
                        const float ceil_v = g_od.max_velocity_rad_s;
                        const float ceil_a = g_od.max_accel_rad_s2;
                        if (ceil_v > 0.001f && vmax > ceil_v) { vmax = ceil_v; }
                        if (ceil_a > 0.001f && amax > ceil_a) { amax = ceil_a; }
                        if (ceil_a > 0.001f && dmax > ceil_a) { dmax = ceil_a; }
                        /* Fall back to safe defaults if the profile limits are unset (0): a move then
                           still plans rather than failing INVALID_LIMITS (which would just hold). */
                        req.limits.max_velocity_rad_per_s      = (vmax > 0.001f) ? vmax :  2.0f;
                        req.limits.max_acceleration_rad_per_s2 = (amax > 0.001f) ? amax : 10.0f;
                        req.limits.max_deceleration_rad_per_s2 = (dmax > 0.001f) ? dmax : 10.0f;
                    }
                    req.limits.max_jerk_rad_per_s3         = 0.0f;
                    MC_PositionController_Reset(&s_pos_ctl);
                    (void)MC_Trajectory_Start(&s_traj, &req);
                }
            }
            else if (ds.active_mode == MC_MODE_POSITION_HOLD)
            {
                /* HALT (ADR-035): controlled hold -- stay enabled, hold the position captured when
                   HALT engaged. On entry, abandon any in-progress move (stay enabled, unlike
                   quick-stop). New setpoints are ignored (the trajectory-start path lives only under
                   MC_MODE_PROFILE_POSITION). Resume by clearing HALT: the trajectory is inactive, so
                   D3 holds at s_pos_hold_rad until a fresh NEW_SETPOINT. */
                s_eff_torque_mode   = false;
                s_eff_position_mode = true;
                if (halt_rise)
                {
                    s_traj.active  = false;
                    s_pos_hold_rad = s_est.mechanical.position_rad - s_home_offset_rad;
                    MC_PositionController_Reset(&s_pos_ctl);
                }
            }
            else
            {
                s_eff_drive         = false;   /* disabled / quick-stop -> safe */
                s_eff_position_mode = false;
            }
            if (dc.fault_reset) { g_mc_inject.clear_fault = true; }   /* clear oc_trip in the fast loop */
            g_od.statusword    = ds.statusword;
            g_od.modes_of_operation_display = g_od.modes_of_operation;
        }
    }

    /* Electrical-alignment routine (ADR-024). Runs AFTER the arbiter and OVERRIDES the effective
       command while active: open-loop d-axis voltage at electrical angle 0, with Vd regulated by a
       slow current-magnitude integrator so the d-axis current (= phase A current at the forced
       angle) reaches cal_align_current_a -- no FOC/angle dependency. Holds for cal_align_hold_ms,
       then captures the electrical offset, safe-offs and saves. Aborts to safe-off on over-current. */
    if (g_mc_inject.request_align_routine)
    {
        g_mc_inject.request_align_routine = false;
        if ((s_align_state == MC_ALIGN_IDLE) && !s_eff_drive && !s_oc_trip)
        {
            s_align_state      = MC_ALIGN_RUN;
            s_align_ticks_left = g_od.cal_align_hold_ms;   /* 1 kHz medium loop -> 1 ms/tick */
            s_align_vd         = 0.0f;
            g_od.cal_status    = MC_IF_CAL_ALIGN_CAPTURE;  /* in progress */
        }
        else
        {
            g_od.cal_status = MC_CAL_STATUS_FAULT;         /* rejected: drive active / busy / fault */
        }
    }
    if (s_align_state == MC_ALIGN_RUN)
    {
        if (s_oc_trip)
        {
            s_align_state   = MC_ALIGN_IDLE;               /* abort -> safe-off */
            s_eff_align     = false;
            s_align_vd      = 0.0f;
            g_od.cal_status = MC_CAL_STATUS_FAULT;
        }
        else
        {
            const float i_target = MC_Math_Clamp(g_od.cal_align_current_a, 0.0f,
                                                 0.9f * g_mc_inject.current_limit_a);  /* under the trip */
            const float i_d      = s_currents.ia_a;        /* = id at the forced electrical angle 0 */
            s_align_vd += MC_ALIGN_KI_V_PER_A * (i_target - i_d);
            s_align_vd  = MC_Math_Clamp(s_align_vd, 0.0f, MC_C2_VD_MAX);

            s_eff_align       = true;
            s_eff_align_v     = s_align_vd;
            s_eff_align_angle = 0.0f;
            s_eff_drive       = false;
            s_eff_torque_mode = false;

            if (s_align_ticks_left > 0u)
            {
                s_align_ticks_left--;
            }
            else
            {
                /* Hold complete: capture the offset at the now-aligned rotor, safe-off, save. */
                s_est_cfg.electrical_offset_rad =
                    MC_Math_Wrap2Pi(-s_pos_sample.position_rad * s_est_cfg.pole_pairs);
                g_mc_debug.elec_offset_rad = s_est_cfg.electrical_offset_rad;
                s_eff_align     = false;
                s_align_vd      = 0.0f;
                s_align_state   = MC_ALIGN_IDLE;
                g_od.cal_status = MC_IF_CAL_NONE;          /* done */
                params_save();
            }
        }
    }

    /* Loop-tuning test-signal overlay (ADR-030). When a tuning mode is armed AND its operational loop
       is enabled, an on-motor generator drives that loop's reference. The output is applied to the
       latched loop while the generator is active, so disarming ramps the reference bumplessly to 0. */
    {
        const bool vel_ok = s_eff_drive && !s_eff_torque_mode && !s_eff_position_mode && !s_oc_trip;
        const bool pos_ok = s_eff_drive &&  s_eff_position_mode && !s_oc_trip;
        const bool cur_ok = s_eff_drive &&  s_eff_torque_mode && !s_oc_trip;   /* current/torque tuning (ADR-030) */
        const uint8_t want = ((g_od.test_mode == MC_IF_TEST_MODE_VELOCITY) && vel_ok) ? MC_IF_TEST_MODE_VELOCITY
                           : ((g_od.test_mode == MC_IF_TEST_MODE_POSITION) && pos_ok) ? MC_IF_TEST_MODE_POSITION
                           : ((g_od.test_mode == MC_IF_TEST_MODE_CURRENT)  && cur_ok) ? MC_IF_TEST_MODE_CURRENT
                           :  MC_IF_TEST_MODE_OFF;

        if (g_mc_inject.request_test_fire)
        {
            g_mc_inject.request_test_fire = false;
            if (want != MC_IF_TEST_MODE_OFF)
            {
                if (want == MC_IF_TEST_MODE_POSITION)
                {
                    s_pos_tune_entry_rad = s_est.mechanical.position_rad - s_home_offset_rad;
                }
                /* Accel limit applies to position tuning only (ADR-032); velocity tuning's rate is
                   already its acceleration, so pass 0 (linear ramp) there. */
                const float accel_lim = (want == MC_IF_TEST_MODE_POSITION) ? g_od.test_max_accel : 0.0f;
                MC_SignalGen_Start(&s_sig_gen, g_od.test_amplitude, g_od.test_rate,
                                   g_od.test_dwell_s, g_od.test_pause_s, accel_lim,
                                   g_od.test_continuous != 0u);
                s_sig_loop = want;
            }
        }
        if ((want == MC_IF_TEST_MODE_OFF) && MC_SignalGen_Active(&s_sig_gen))
        {
            MC_SignalGen_Stop(&s_sig_gen);   /* disarmed / conditions lost -> ramp to 0 */
        }

        s_sig_value = MC_SignalGen_Update(&s_sig_gen, MC_MOTION_DT_S);
        if (!MC_SignalGen_Active(&s_sig_gen)) { s_sig_loop = MC_IF_TEST_MODE_OFF; }

        /* Velocity-tuning: the generator IS the velocity-loop demand (position-tuning is applied in D3). */
        if (MC_SignalGen_Active(&s_sig_gen) && (s_sig_loop == MC_IF_TEST_MODE_VELOCITY) && vel_ok)
        {
            s_eff_vel_cmd = s_sig_value;
        }
        /* Current-tuning: the generator IS the current/torque command [A] (overrides the arbiter's iq).
           amplitude (0x2910:2) is the requested current; rate 0 -> a step pulse. ADR-030. */
        if (MC_SignalGen_Active(&s_sig_gen) && (s_sig_loop == MC_IF_TEST_MODE_CURRENT) && cur_ok)
        {
            s_eff_iq_cmd = s_sig_value;
        }
        g_od.test_active = MC_SignalGen_Active(&s_sig_gen) ? 1u : 0u;
        g_od.test_signal = s_sig_value;   /* 0x2910:8 PDO -- the generator output, for graphing */
    }

    /* Stage D3: position cascade (ADR-028). In PROFILE_POSITION, advance the trajectory and run the
       position loop; the result is a velocity demand the D2 stage below executes (so all velocity/
       torque limits + the over-current trip still apply). */
    if (s_eff_drive && s_eff_position_mode && !s_oc_trip)
    {
        const float p_act = s_est.mechanical.position_rad - s_home_offset_rad;   /* home-relative */
        if (!s_pos_on)
        {
            MC_PositionController_Reset(&s_pos_ctl);
            s_pos_hold_rad = p_act;   /* latch the current position to hold until a move is commanded */
            s_pos_on = true;
        }

        float p_dem, v_ff, a_ff;
        bool  complete;
        if (s_eff_halt)
        {
            /* HALT hold (ADR-035): hold at the position captured when HALT engaged (move abandoned
               in the arbiter). Pure feedback -- no FF, no trajectory. */
            p_dem = s_pos_hold_rad; v_ff = 0.0f; a_ff = 0.0f; complete = true;
        }
        else if (MC_SignalGen_Active(&s_sig_gen) && (s_sig_loop == MC_IF_TEST_MODE_POSITION))
        {
            /* Position-tuning (ADR-030): generated reference around the captured entry position,
               bypassing the trajectory. The generator emits its velocity, used as the FF (ADR-031) --
               so a ramp feeds forward ±rate; a step (rate 0) has vel 0, i.e. pure feedback. */
            p_dem = s_pos_tune_entry_rad + s_sig_value;
            v_ff = MC_SignalGen_Velocity(&s_sig_gen); a_ff = 0.0f; complete = false;
            s_pos_hold_rad = p_dem;   /* hold here when the test ends */
        }
        else
        {
            /* Use the active plan if there is one; otherwise HOLD the latched position. A freshly-init
               or failed planner evaluates to position 0 (sp.valid == false) -- driving to it would slam
               the axis to home (the "only moves to 0" bug). */
            const MC_MotionSetpoint_t sp = MC_Trajectory_Update(&s_traj, MC_MOTION_DT_S);
            if (sp.valid)
            {
                p_dem = sp.position_rad; v_ff = sp.velocity_rad_per_s; a_ff = sp.acceleration_rad_per_s2;
                complete = sp.complete; s_pos_hold_rad = sp.position_rad;
            }
            else
            {
                p_dem = s_pos_hold_rad; v_ff = 0.0f; a_ff = 0.0f; complete = true;
            }
        }

        const float vcorr = MC_PositionController_Update(&s_pos_ctl, &s_pos_cfg, p_dem, p_act);
        s_eff_vel_cmd     = (s_vel_ff_gain * v_ff) + vcorr;  /* velocity demand = FF-gain·FF + position correction (ADR-031) */
        s_accel_ff_rad_s2 = a_ff;             /* -> torque request inertia slot */

        const float perr = s_pos_ctl.position_error_rad;
        const bool reached = complete && (perr < MC_POS_TARGET_WINDOW_RAD) && (perr > -MC_POS_TARGET_WINDOW_RAD);
        if (reached) { g_od.statusword |= MC_IF_SW_TARGET_REACHED; }

        g_mc_debug.pos_demand_rad = p_dem;
        g_mc_debug.pos_actual_rad = p_act;
        g_mc_debug.pos_error_rad  = perr;
        g_mc_debug.target_reached = reached;
    }
    else
    {
        s_pos_on          = false;
        s_accel_ff_rad_s2 = 0.0f;
        g_mc_debug.target_reached = false;
    }

    /* Stage D2: velocity cascade -> torque request -> iq, published to the fast loop. */
    const bool vel_active = s_eff_drive && !s_eff_torque_mode && !s_oc_trip;
    if (vel_active)
    {
        if (!s_vel_on) { MC_VelocityController_Reset(&s_vel); s_vel_on = true; }

        /* Motor safety envelope (ADR-040): clamp the velocity demand to the motor-owned ceiling,
           whatever its source (position cascade, direct velocity, signal generator). 0 = disabled. */
        float vdem = s_eff_vel_cmd;
        {
            const float ceil_v = g_od.max_velocity_rad_s;
            if (ceil_v > 0.001f)
            {
                if      (vdem >  ceil_v) { vdem =  ceil_v; }
                else if (vdem < -ceil_v) { vdem = -ceil_v; }
            }
        }
        /* Soft position limits (ADR-040): don't drive further past a manually-set limit. lo>=hi = disabled. */
        {
            const float lo = g_od.pos_limit_lo_rad, hi = g_od.pos_limit_hi_rad;
            if (hi > lo)
            {
                const float pos_rel = s_est.mechanical.position_rad - s_home_offset_rad;
                if (pos_rel >= hi && vdem > 0.0f) { vdem = 0.0f; }
                if (pos_rel <= lo && vdem < 0.0f) { vdem = 0.0f; }
            }
        }
        const float vact = s_est.mechanical.velocity_rad_per_s;   /* observer by default */
        const float tcorr = MC_VelocityController_Update(&s_vel, &s_vel_cfg, vdem, vact);

        MC_CurrentRequestDebug_t crd;
        MC_MotorTorqueRequest_t treq =
            MC_CurrentRequest_Update(&s_torque_cfg, tcorr, s_accel_ff_rad_s2 /* trajectory accel FF (D3) */, vact, true, &crd);
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
    if (g_od.cal_command == MC_IF_CAL_ALIGN_CAPTURE)
    {
        g_mc_inject.request_align_routine = true;     /* medium loop runs the alignment routine */
        g_od.cal_command = MC_IF_CAL_NONE;
    }
    /* Current-offset calibration (ADR-026): the fast loop averages zero-current ADC samples, so it
       is only valid with the power stage off. Accept when PWM is off; otherwise reject. */
    if (g_od.cal_command == MC_IF_CAL_CURRENT_OFFSET)
    {
        g_od.cal_command = MC_IF_CAL_NONE;
        if (!s_pwm_on)
        {
            g_mc_inject.request_offset_cal = true;        /* fast loop runs MC_CurrentSense_CalibrateOffsets */
            g_od.cal_status = MC_IF_CAL_CURRENT_OFFSET;   /* accepted / in progress */
        }
        else
        {
            g_od.cal_status = MC_CAL_STATUS_FAULT;        /* rejected: drive / PWM active */
        }
    }
    /* Current-offset cal finished once the fast loop clears the request -> report done and
       auto-save the freshly measured offsets (matches alignment / set-mech-zero, ADR-026). */
    if ((g_od.cal_status == MC_IF_CAL_CURRENT_OFFSET) && !g_mc_inject.request_offset_cal)
    {
        g_od.cal_status = MC_IF_CAL_NONE;
        params_save();
    }

    /* Loop-tuning test-signal trigger (0x2910:6 -> fire the generator; ADR-030). */
    if (g_od.test_trigger != 0u)
    {
        g_od.test_trigger = 0u;
        g_mc_inject.request_test_fire = true;
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
