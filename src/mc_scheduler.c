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
#include "mc_traj_scurve.h" /* jerk-limited S-curve planner, selectable via 0x2600:9 (ADR-045) */
#include "mc_freq_sweep.h"   /* stepped-sine current sweep for resonance ID, fast-loop injected (ADR-047) */
#include "mc_notch.h"        /* band-reject on the current command for resonance suppression (ADR-048) */
#include "mc_thermal.h"      /* winding I²t thermal model + progressive current-limit derate (ADR-065) */
#include "mc_dither.h"       /* low-speed anti-stiction current dither (ADR-066) */
#include "mc_pos_recall.h"   /* persistent last-position journal for non-back-drivable incremental axes (ADR-067) */
#include "mc_homing.h"       /* home-to-hard-stop sequencer, extracted from this file (ADR-068) */
#include "mc_quad_encoder.h" /* TIM2 quadrature count, mirrored to 0x2510:4 (ADR-050) */
#include "mc_position_controller.h"
#include "mc_signal_gen.h"
#include "mc_od.h"
#include "mc_od_store.h"
#include "mc_comms.h"
#include "mc_boot_meta.h"  /* dual-bootloader healthy-window flag clear (REQ-0015) */
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

/* The persisted parameter set must fit the store payload (ADR-070). If this fails, raise
   MC_PARAM_STORE_MAX_PAYLOAD (the flash slot is 2 KB, so there is room) -- do NOT shrink the blob. */
_Static_assert(sizeof(MC_Params_t) <= MC_PARAM_STORE_MAX_PAYLOAD,
               "MC_Params_t exceeds the persistent-store payload cap");

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
#define MC_C2_VD_MAX 12.0f  /* clamp on the open-loop d-axis voltage [V] -- shared by alignment, manual commissioning,
                               and the plant-ID pulse. ~Vbus/2 SVPWM ceiling at 24 V (duty saturates beyond, OC trip
                               bounds current). Raised from 3 V for plant ID (ADR-046). */
#define MC_DQ_TEST_MAX_MS 10000u  /* d-axis plant-ID max dwell / auto-disarm backstop [ms] @ 1 kHz arbitration (ADR-046) */
#define MC_STORE_LOAD_ATTEMPTS 3u  /* persistent-config load retries at boot before failing safe (ADR-051) */
#define MC_JOG_LEASH_RAD   1.0f    /* position-integrated jog: the moving reference may lead the actual by at most this [rad] (ADR-062) */
/* Homing timing constants (MC_HOME_*) moved into mc_homing.c with the sequencer (ADR-068). */
#define MC_RECALL_SETTLE_TICKS 100u  /* position-recall settle dwell: MOVING clear this long before storing [100 Hz slow -> 1 s] (ADR-067) */
#define MC_RECALL_MOVE_EPS 0.01f     /* |mechanical velocity| below this counts as "not moving" for recall [rad/s] (ADR-067) */
#define MC_RECALL_STORE_EPS 0.001f   /* store-on-change: skip a re-store if the settled position moved less than this [rad] (ADR-067) */
static bool s_oc_trip;      /* latched over-current trip */
static uint32_t s_fault_flags_prev; /* previous fault_flags -> per-fault rising-edge counts (ADR-058) */
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
static float s_quad_rad_per_count;    /* signed 2pi/counts_per_rev for the incremental quad (ADR-052) */

/* Brushed-DC backend (ADR-039): single armature-current PI -> locked anti-phase H-bridge voltage. */
static MC_Pid_t       s_hb_ipi;
static MC_PidConfig_t s_hb_ipi_cfg;

/* Stage D2: velocity loop + torque/current request (runs in the medium loop). */
static MC_MotorModel_t               s_motor;
static MC_VelocityController_t       s_vel;
static MC_VelocityControllerConfig_t s_vel_cfg;
static MC_TorqueModelConfig_t        s_torque_cfg;
static volatile float                s_iq_cmd_published;  /* velocity-loop iq, medium->fast (atomic) */
static volatile float                s_i_demand_max_a;    /* soft max demanded current (0x2400:8, 0=off); slow->fast (atomic) (ADR-069) */
static bool                          s_vel_on;            /* velocity loop active (for entry reset) */
static MC_Homing_t                   s_homing;            /* home-to-hard-stop sequencer state (ADR-057/068) */
static bool                          s_set_zero_at_pending; /* deferred SET_MECH_ZERO_AT: apply mech_zero_set_rad in the medium loop (ADR-022) */
static bool                          s_homed;             /* incremental encoder zeroed this power-cycle (NOT persisted; gates position recalls) (ADR-057) */

/* Persistent position recall (ADR-067): incremental-axis last-position journal. */
static bool                          s_recall_applied;      /* startup recall one-shot consumed (checked once, after the first sample) */
static bool                          s_recall_used;         /* startup recall actually adopted a stored position (drives 0x2700:12 status) */
static bool                          s_recall_was_moving;   /* previous MOVING state for the store/invalidate edge detector */
static uint16_t                      s_recall_settle_ticks; /* slow-loop ticks MOVING has stayed clear (settle dwell) */

/* Stage D3: trajectory + position loop (runs in the medium loop; feeds the velocity cascade). */
static MC_TrajectoryPlanner_t        s_traj;
static MC_PositionController_t       s_pos_ctl;
static MC_PositionControllerConfig_t s_pos_cfg;
static bool                          s_eff_position_mode; /* PROFILE_POSITION active (medium-loop arbiter) */
static bool                          s_pos_on;            /* position loop active (for entry reset) */
static float                         s_accel_ff_rad_s2;   /* trajectory accel feedforward -> torque request */
static float                         s_vel_ff_gain;       /* velocity FF ratio in the position cascade (0x2200:4, ADR-031) */
static float                         s_jog_ref_vel;       /* position-jog reference velocity (Δref/dt) -> velocity FF (ADR-073); 0 when not jogging */
static float                         s_pos_hold_rad;      /* held position when in position mode with no active plan */

/* Loop-tuning test-signal overlay (ADR-030): an on-motor generator drives the selected loop's reference. */
static MC_SignalGen_t                s_sig_gen;
static MC_FreqSweep_t                s_freq_sweep;         /* stepped-sine current sweep (ADR-047) */
static MC_Notch_t                    s_iq_notch;           /* current-command notch (ADR-048) */
static float                         s_notch_last_f0 = -1.0f, s_notch_last_bw = -1.0f;  /* coeff-recompute guard */
static uint8_t                       s_sig_loop;           /* latched target loop while active (MC_IF_TEST_MODE_*) */
static float                         s_sig_value;          /* generator output this medium tick */
static float                         s_pos_tune_entry_rad; /* position captured when position-tuning fires (home-relative) */
#define MC_POS_TARGET_WINDOW_RAD (0.01f)                  /* target-reached window FALLBACK when position_deadband_rad is 0 (ADR-076); else the deadband is the window */

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
static volatile bool  s_eff_align_q;      /* open-loop test on the q-axis (else d-axis) (ADR-046 ext) */
static volatile bool  s_eff_hb_test;      /* open-loop brushed armature voltage test active (ADR-046 ext) */
static volatile float s_eff_hb_test_v;    /* open-loop brushed armature voltage [V] */
static uint32_t       s_dq_test_ticks;    /* d-axis plant-ID arm-time counter (auto-disarm, ADR-046) */

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
/* --- Velocity-demand acceleration ramp (jerk-limited slew, ADR-042) ------------------------------
   Ramps the PROFILE_VELOCITY demand (the joystick path) toward the setpoint under an acceleration cap,
   where the acceleration itself eases IN (jerk-limited on the way UP to the cap) but cuts off FREELY on
   the way DOWN. That asymmetry is what makes it stable: the acceleration can always fall in time to land
   the velocity on the setpoint (no overshoot), while the soft rise removes the start kick. The position
   cascade and the tuning generator bypass it. Runs at the medium-loop rate.
     - accel_up [rad/s^2]  : max acceleration while speeding up (|v| growing); accel_dn while slowing down.
                             0 disables the limiter for that phase (pass-through).
     - accel_jerk [rad/s^3]: how fast the acceleration ramps UP to the cap (shared). 0 = step (no soft rise).
                             The down direction has no knob -- the acceleration magnitude falls without limit. */
static float s_vel_accel_up;     /* 0x2300:6 [rad/s^2] -- applied from the OD in od_apply_gains */
static float s_vel_accel_dn;     /* 0x2300:7 [rad/s^2] */
static float s_vel_accel_jerk;   /* 0x2300:8 [rad/s^3] -- accel ramp-up rate (0 = step) */
static bool  s_vel_accel_scurve; /* 0x2300:14 -- 1 = anticipatory jerk-limited S-curve (rounds both ends, ADR-075); 0 = ramp-up-only free-fall */
static float s_slew_vel_prev;    /* limiter state: last output velocity demand [rad/s] */
static float s_slew_acc_prev;    /* limiter state: last applied acceleration [rad/s^2] */

static void vel_slew_reset(float vel)
{
    s_slew_vel_prev = vel;
    s_slew_acc_prev = 0.0f;
}

static float vel_slew_limit(float vel_in)
{
    const float dt   = MC_MOTION_DT_S;
    const float alim = (fabsf(vel_in) >= fabsf(s_slew_vel_prev)) ? s_vel_accel_up : s_vel_accel_dn;

    if (alim <= 0.0f)                       /* limiter disabled for this phase -> pass-through */
    {
        s_slew_vel_prev = vel_in;
        s_slew_acc_prev = 0.0f;
        return vel_in;
    }

    /* Anticipatory jerk-limited S-curve (ADR-075). Rounds BOTH ends of the velocity ramp: the
       acceleration is held on the phase-plane braking boundary a = sqrt(2*j*|e|) (e = remaining
       velocity), so it eases to zero exactly as the velocity reaches the target -- no overshoot and,
       unlike the free-fall path below, no acceleration discontinuity at the setpoint. Re-planned
       every tick against the live target, so a moving joystick just re-tracks (jerk-bounded); the
       backstop guarantees no overshoot even mid-tune. Reuses accel_jerk (0x2300:8) as the jerk cap. */
    if (s_vel_accel_scurve && (s_vel_accel_jerk > 0.0f))
    {
        const float j = s_vel_accel_jerk;
        const float e = vel_in - s_slew_vel_prev;               /* remaining velocity to the target */
        const float a_stop = sqrtf(2.0f * j * fabsf(e));        /* accel from which we can still stop within |e| */
        const float a_cap  = (a_stop < alim) ? a_stop : alim;   /* capped by the phase-plane AND accel_up/dn */
        const float a_tgt  = (e >= 0.0f) ? a_cap : -a_cap;

        float a  = s_slew_acc_prev;
        float da = a_tgt - a;                                   /* ramp the applied accel toward the target */
        const float jerk_dt = j * dt;                           /* ...at the jerk limit */
        if      (da >  jerk_dt) { da =  jerk_dt; }
        else if (da < -jerk_dt) { da = -jerk_dt; }
        a += da;

        float vel_out = s_slew_vel_prev + a * dt;
        if (vel_in >= s_slew_vel_prev) { if (vel_out > vel_in) { vel_out = vel_in; } }  /* backstop: never cross */
        else                           { if (vel_out < vel_in) { vel_out = vel_in; } }
        s_slew_acc_prev = (vel_out - s_slew_vel_prev) / dt;     /* store the acceleration actually applied */
        s_slew_vel_prev = vel_out;
        return vel_out;
    }

    /* Acceleration to land exactly on the demand this tick, capped at the phase's max acceleration. */
    float acc_target = (vel_in - s_slew_vel_prev) / dt;
    if      (acc_target >  alim) { acc_target =  alim; }
    else if (acc_target < -alim) { acc_target = -alim; }

    /* Move the applied acceleration toward acc_target: its magnitude may RISE by <= jerk*dt per tick,
       but may FALL without limit -- the asymmetry that prevents overshoot. jerk <= 0 => step to target. */
    const float acc_prev = s_slew_acc_prev;
    float acc;
    if (s_vel_accel_jerk <= 0.0f)
    {
        acc = acc_target;                                       /* no soft rise: step to the capped target */
    }
    else
    {
        const float jerk_dt = s_vel_accel_jerk * dt;
        if (acc_target * acc_prev < 0.0f)                       /* sign flip: fall to 0 free, then limited rise */
        {
            acc = (acc_target >  jerk_dt) ?  jerk_dt
                : (acc_target < -jerk_dt) ? -jerk_dt
                :  acc_target;
        }
        else                                                    /* same side of 0 (or rising from 0) */
        {
            const float mag_prev = fabsf(acc_prev);
            if (fabsf(acc_target) > mag_prev + jerk_dt)         /* rising magnitude: jerk-limit the rise */
            {
                acc = (acc_target >= 0.0f) ? (mag_prev + jerk_dt) : -(mag_prev + jerk_dt);
            }
            else                                                /* falling, or rising within budget: take it */
            {
                acc = acc_target;
            }
        }
    }

    float vel_out = s_slew_vel_prev + acc * dt;

    /* Backstop: never cross the demand (a capped accel can't, but a ramp-in from a stale state might). */
    if (vel_in >= s_slew_vel_prev) { if (vel_out > vel_in) { vel_out = vel_in; } }
    else                           { if (vel_out < vel_in) { vel_out = vel_in; } }

    s_slew_acc_prev = (vel_out - s_slew_vel_prev) / dt;         /* acceleration actually applied */
    s_slew_vel_prev = vel_out;
    return vel_out;
}

/* Soft position limits are home-relative, so they're meaningless until the mechanical zero is set.
   Active = a real band (lo<hi) AND the zero captured. s_home_offset_rad != 0 is the same "mech zero
   done" signal as MC_IF_CAL_DONE_MECH_ZERO (the cal_done bitfield, ADR-040/043). */
static bool pos_limits_active(void)
{
    return (g_od.pos_limit_hi_rad > g_od.pos_limit_lo_rad) && (s_home_offset_rad != 0.0f);
}

static void od_apply_gains(void)
{
    const float kt   = g_od.motor_kt_nm_per_a;
    /* Thermal derate (ADR-065): scale the operational current limit by the model's derate
       factor (1.0 when the model is disabled or cool). The hard OC trip (0x2600:2, applied
       below) is deliberately NOT derated -- it stays the absolute safety backstop. */
    const float i_lim = g_od.vel_current_limit_a * MC_Thermal_DerateFactor();
    const float tlim  = i_lim * kt;

    {
        /* vel_load_factor (0x2300:5, REQ-0014/ADR-034): operator load multiplier on the velocity-loop
           gains, clamped to [0.3, 2.0] so a stray/zero write can't kill or blow up the loop. */
        const float lf = (g_od.vel_load_factor < 0.3f) ? 0.3f
                       : (g_od.vel_load_factor > 2.0f) ? 2.0f : g_od.vel_load_factor;
        s_vel_cfg.pid.kp = g_od.vel_kp * lf;
        s_vel_cfg.pid.ki = g_od.vel_ki * lf;
    }
    s_vel_cfg.pid.kd = g_od.vel_kd;
    s_vel_accel_up   = g_od.vel_accel_up;   /* velocity-demand acceleration ramp (0x2300:6/7/8, ADR-042) */
    s_vel_accel_dn   = g_od.vel_accel_dn;
    s_vel_accel_jerk = g_od.vel_accel_jerk;
    s_vel_accel_scurve = (g_od.vel_accel_scurve != 0u);   /* 0x2300:14 anticipatory jerk-limited ramp (ADR-075) */
    s_vel_cfg.pid.output_min     = -tlim;  s_vel_cfg.pid.output_max     = tlim;
    s_vel_cfg.pid.integrator_min = -tlim;  s_vel_cfg.pid.integrator_max = tlim;
    s_vel_cfg.torque_output_limit_nm = tlim;
    s_vel_cfg.stop_bleed_enable = (g_od.vel_stop_bleed_enable != 0u);  /* 0x2300:13 stop-integrator bleed on/off (ADR-074) */
    s_vel_cfg.stop_bleed_v_th   = g_od.vel_stop_bleed_v_th;    /* 0x2300:11 bleed threshold [rad/s] (ADR-074) */
    s_vel_cfg.stop_bleed_factor = g_od.vel_stop_bleed_factor;  /* 0x2300:12 bleed speed as a factor of ki (ADR-074) */
    s_torque_cfg.current_limit_a        = i_lim;   /* thermally-derated operational limit (ADR-065) */
    s_torque_cfg.torque_limit_nm        = tlim;
    s_torque_cfg.torque_constant_nm_per_a = kt;

    /* Over-current trip threshold (measured |phase current|, fast loop): driven by the OD entry
       current_trip_a (0x2600:2) -- GUI-settable + PERSIST. Clamp to a small positive minimum so a
       stray 0 / negative can't latch the trip permanently and lock the drive out. (ADR-029) */
    g_mc_inject.current_limit_a = (g_od.current_trip_a > 0.1f) ? g_od.current_trip_a : 0.1f;

    /* Soft max-demand current ceiling (0x2400:8, ADR-069): a working limit below the OC trip,
       applied at the current-loop input in the fast loop. 0 = disabled. */
    s_i_demand_max_a = g_od.current_demand_limit_a;

    /* Position loop (D3, ADR-028): P-default gains (0x2200); velocity correction capped at the
       profile velocity (0x6081), falling back to 10 rad/s if unset. */
    s_pos_cfg.pid.kp = g_od.pos_kp;
    s_pos_cfg.pid.ki = g_od.pos_ki;
    s_pos_cfg.pid.kd = g_od.pos_kd;
    s_pos_cfg.deadband_rad = g_od.position_deadband_rad;   /* 0x2200:5 position-error deadband (ADR-071) */
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
    /* Brushed current loop: kp/ki are set DIRECTLY from the OD (0x2400:6,7, RW PERSIST), hand-tuned
       (ADR-049, replacing the R/L+bandwidth derivation). R/L (0x2000:3,4) still feed the model. */
    s_motor.resistance_ohm = g_od.motor_resistance_ohm;
    s_motor.inductance_h   = g_od.motor_inductance_h;
    s_hb_ipi_cfg.kp = g_od.hb_cur_kp;
    s_hb_ipi_cfg.ki = g_od.hb_cur_ki;
    /* Incremental quad scale: signed rad/count = 2pi / counts_per_rev (the sign sets direction). (ADR-052) */
    s_quad_rad_per_count = (fabsf(g_od.quad_counts_per_rev) > 1.0f)
                         ? (6.28318530717958648f / g_od.quad_counts_per_rev) : 0.0f;

    s_est_cfg.velocity_filter_hz = g_od.est_velocity_filter_hz;
    if ((g_od.notch_freq_hz != s_notch_last_f0) || (g_od.notch_bandwidth_hz != s_notch_last_bw))
    {   /* recompute the current-command notch coefficients only when the band changes (ADR-048) */
        MC_Notch_SetParams(&s_iq_notch, g_od.notch_freq_hz, g_od.notch_bandwidth_hz, 1.0f / MC_MOTION_DT_S);
        s_notch_last_f0 = g_od.notch_freq_hz; s_notch_last_bw = g_od.notch_bandwidth_hz;
    }
    /* current_trip stays on the watch-window inject path during bring-up (read-reflected in
       od_mirror_live), to avoid a two-writer conflict. */
}

/* Mirror live state into the OD store so reads return current values (telemetry RO; observer
   gains / electrical offset reflect their live source). */
static void od_mirror_live(void)
{
    g_od.tlm_vel_demand_rad_s     = g_mc_debug.vel_demand_rad_s;
    g_od.quad_encoder_count       = MC_QuadEnc_Count();   /* TIM2 quadrature count -> 0x2510:4 (ADR-050) */
    g_od.tlm_vel_actual_rad_s     = g_mc_debug.mech_velocity_rad_s;
    g_od.tlm_vel_iq_cmd_a         = g_mc_debug.vel_iq_cmd_a;
    g_od.tlm_id_meas_a            = g_mc_debug.id_meas_a;
    g_od.tlm_iq_meas_a            = g_mc_debug.iq_meas_a;
    g_od.tlm_vd_v                 = g_mc_debug.vd_v;
    g_od.tlm_vq_v                 = g_mc_debug.vq_v;
    g_od.tlm_electrical_angle_rad = g_mc_debug.elec_angle_rad;
    g_od.tlm_i_arm_a              = g_mc_debug.i_arm_a;   /* brushed armature current (0x2410:6) */
    g_od.tlm_v_arm_v              = g_mc_debug.v_cmd_v;   /* brushed armature voltage cmd (0x2410:7, the vq analog) */
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
        /* Soft position limits (ADR-040/043): flag AT_LIMIT_LO/HI when at/past a manually-set limit.
           Gated on pos_limits_active() = a real band AND the mechanical zero set (home-relative). */
        if (pos_limits_active())
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
    g_od.fault_flags     = (MC_PersistentStore_HasValid() ? 0u : MC_IF_FAULT_NO_CONFIG)   /* ADR-051 */
                         | ((!s_pos_sample.absolute && !s_homed) ? MC_IF_FAULT_NOT_HOMED : 0u)  /* ADR-057 */
                         | (s_oc_trip ? MC_IF_FAULT_OVERCURRENT : 0u)                     /* ADR-058: OC trip as a fault bit */
                         | (MC_Thermal_OverTemp() ? MC_IF_FAULT_OVERTEMP : 0u);          /* ADR-065: thermal backstop */
    g_od.fault_flags_latched |= g_od.fault_flags;   /* sticky since-boot fault history (ADR-058) */
    /* Per-fault since-boot trigger counts: bump on each fault bit's RISING edge (saturating U16). */
    const uint32_t fault_rising = g_od.fault_flags & ~s_fault_flags_prev;
    if ((fault_rising & MC_IF_FAULT_NO_CONFIG)   && g_od.fault_count_no_config   != 0xFFFFu) { g_od.fault_count_no_config++; }
    if ((fault_rising & MC_IF_FAULT_NOT_HOMED)   && g_od.fault_count_not_homed   != 0xFFFFu) { g_od.fault_count_not_homed++; }
    if ((fault_rising & MC_IF_FAULT_OVERCURRENT) && g_od.fault_count_overcurrent != 0xFFFFu) { g_od.fault_count_overcurrent++; }
    if ((fault_rising & MC_IF_FAULT_OVERTEMP)    && g_od.fault_count_overtemp    != 0xFFFFu) { g_od.fault_count_overtemp++; }
    s_fault_flags_prev = g_od.fault_flags;
    /* motor_resistance/inductance (0x2000:3,4) are now config inputs (applied in od_apply_gains),
       no longer mirrored from the model here -- writing them sticks (ADR-039 R/L promotion). */
    g_od.store_status    = (uint16_t)((MC_PersistentStore_HasValid()   ? MC_IF_STORE_VALID   : 0u)
                                    | (MC_PersistentStore_SavePending() ? MC_IF_STORE_PENDING : 0u));

    /* Position-recall status (0x2700:12, ADR-067): 0 off/N-A, 1 recalled-valid (homing skipped),
       2 stale -> homing required (latest record INVALID = power lost mid-move), 3 nothing stored. */
    if ((g_od.position_recall_enable == 0u) || s_pos_sample.absolute) { g_od.position_recall_status = 0u; }
    else if (s_recall_used)                                           { g_od.position_recall_status = 1u; }
    else if (MC_PosRecall_HasAnyRecord())                             { g_od.position_recall_status = 2u; }
    else                                                              { g_od.position_recall_status = 3u; }

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
    g_mc_debug.fw_build = 88u;   /* build/version marker (ADR-038/039/040/042/043/044/045/046/047/048/049/050/051/052/054/056/057/058/061/062): read in the watch window to confirm the flashed image */
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
    MC_FreqSweep_Init(&s_freq_sweep);
    MC_Notch_Init(&s_iq_notch);

    /* Object dictionary: seed defaults (its gains match the configs seeded above). */
    MC_Od_Init();
    MC_Thermal_Init();      /* winding I²t thermal model (ADR-065); safe disabled default */
    MC_Dither_Init();       /* low-speed anti-stiction dither (ADR-066); off by default */
    MC_Homing_Init(&s_homing);   /* home-to-hard-stop sequencer (ADR-057/068) */
    MC_Comms_Init();        /* SPI protocol handler (transport DMA wired in F2b) */
    MC_BootMeta_Init();     /* read boot flag; arm the healthy-window flag clear (REQ-0015) */
    MC_ModeManager_Init();  /* CiA-402 drive state machine (E1) */

    /* Load persisted calibration + gains, retrying so an unlikely transient at cold boot can't silently
       fall back to defaults (the store is already A/B-redundant + CRC-checked; this is belt-and-suspenders,
       ADR-051). If nothing valid loads (cold-boot failure, or a never-configured / version-bumped board),
       MC_PersistentStore_HasValid() stays false -> od_apply_gains raises 0x2600:1 MC_IF_FAULT_NO_CONFIG and
       the medium loop inhibits the operational drive via fs.severe_active (commissioning/align unaffected). */
    for (uint8_t attempt = 0u; (attempt < MC_STORE_LOAD_ATTEMPTS) && !MC_PersistentStore_HasValid(); attempt++)
    {
        MC_Params_t p;
        if ((MC_PersistentStore_Init() == MC_OK) &&
            (MC_PersistentStore_Read(&p, (uint16_t)sizeof p) == MC_OK))
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
        }
    }
    g_mc_debug.store_valid = MC_PersistentStore_HasValid();

    /* Position-recall journal (ADR-067): scan the dedicated POS_RECALL flash region for the latest
       stored position. Whether it is adopted (skipping homing) is decided one-shot in the medium
       loop, once the encoder type + first sample are known. */
    MC_PosRecall_Init();

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

/* Soft max-demand current clamp (ADR-069): bound the commanded (torque-producing) current to
   +/- s_i_demand_max_a, a working limit set below the hard OC trip. 0 = disabled (raw command
   passes through). Applied at the current-loop input for both backends + every command source. */
static inline float clamp_i_demand(float i_cmd)
{
    const float m = s_i_demand_max_a;
    return (m > 0.0f) ? MC_Math_Clamp(i_cmd, -m, m) : i_cmd;
}

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

    /* Frequency-sweep current overlay (ADR-047): edge-detect freq_sweep_enable + generate the stepped-sine
       sample HERE, above the backend dispatch, so it overlays BOTH backends' torque-mode current command
       (was FOC-branch-only, so it never ran on brushed). Sampled once/cycle -> phase advances once. */
    {
        static bool s_sweep_prev_en = false;
        const bool en = (g_od.freq_sweep_enable != 0u);
        if (en && !s_sweep_prev_en)
        {
            MC_FreqSweep_Start(&s_freq_sweep, g_od.freq_sweep_start_hz, g_od.freq_sweep_end_hz,
                               g_od.freq_sweep_step_hz, g_od.freq_sweep_dwell_s,
                               g_od.freq_sweep_bias_a, g_od.freq_sweep_amplitude_a);
        }
        else if (!en && MC_FreqSweep_Active(&s_freq_sweep))
        {
            MC_FreqSweep_Stop(&s_freq_sweep);
        }
        s_sweep_prev_en = en;
    }
    const bool  sweep_on = MC_FreqSweep_Active(&s_freq_sweep) && s_eff_torque_mode;
    const float sweep_iq = sweep_on ? MC_FreqSweep_Sample(&s_freq_sweep, MC_FAST_DT_S) : 0.0f;

    if (brushed && !blocked && (s_eff_drive || s_eff_hb_test))
    {
        /* Armature current for the loop. The new board's ADC1 reads leg B (I_B = -I_A), so s_currents.ia_a
           is the NEGATIVE of the forward armature current -- negate it so the feedback sign matches the
           command. (An inverted measurement is positive feedback: the integrator runs the current away.
           Magnitude was verified vs a meter; the DAC shows |i| and was correct, only the sign was wrong.)
           Dual-leg (I_A - I_B)/2 lands once ADC2 reads IN7 = I_A. */
        const float i_arm = -s_currents.ia_a;
        const float i_cmd = clamp_i_demand(sweep_on ? sweep_iq : (s_eff_torque_mode ? s_eff_iq_cmd : s_iq_cmd_published));  /* sweep overlays here too (ADR-047); demand-limited (ADR-069) */

        /* Open-loop voltage (bring-up: verify current sign/scaling) OR the closed armature-current PI.
           Open-loop holds the PI reset so closing it afterwards is bumpless. Gains are live-tunable. */
        float v_cmd;
        if (s_eff_hb_test)   /* open-loop armature voltage test (brushed_phase, ADR-046 ext) */
        {
            v_cmd = MC_Math_Clamp(s_eff_hb_test_v, -MC_C2_VD_MAX, MC_C2_VD_MAX);
            MC_Pid_Reset(&s_hb_ipi);
        }
        else if (g_mc_inject.hb_open_loop)
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
        /* Torque mode: direct iq, or the frequency-sweep overlay (sweep_on/sweep_iq computed above so it
           overlays both backends). Velocity mode: iq from the medium-loop velocity cascade. (ADR-047) */
        cmd.iq_a   = clamp_i_demand(sweep_on ? sweep_iq : (s_eff_torque_mode ? s_eff_iq_cmd : s_iq_cmd_published));  /* demand-limited (ADR-069) */
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
            /* Commissioning open-loop voltage at the commanded electrical angle (C2). d-axis: V along
               (cos,sin); q-axis: V along (-sin,cos), i.e. 90 deg ahead (ADR-046 ext). */
            float sin_e, cos_e;
            MC_Math_SinCos(s_eff_align_angle, &sin_e, &cos_e);
            const float v_alpha = s_eff_align_q ? (-vd * sin_e) : (vd * cos_e);
            const float v_beta  = s_eff_align_q ? ( vd * cos_e) : (vd * sin_e);
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

    /* Debug DAC (PA4 / DAC1_OUT1): output the signal selected by dac_source (0x2900:5), scaled by
       dac_scale_v_per_a (default 1 V/A -> 1 A = 1 V), clamped to 0..Vref. The DAC is unipolar, so signed
       signals (iq/id/ia/...) clip below 0 -- use the |.| options, or keep the signal positive (e.g. id
       during a d-axis voltage step). */
    {
        float dac_sig;
        switch (g_od.dac_source)
        {
            case 1:  dac_sig = g_mc_debug.iq_meas_a;        break;  /* iq (signed) */
            case 2:  dac_sig = fabsf(g_mc_debug.id_meas_a); break;  /* |id| */
            case 3:  dac_sig = g_mc_debug.id_meas_a;        break;  /* id (signed) */
            case 4:  dac_sig = g_mc_debug.ia_a;             break;  /* phase A (= id at the forced angle 0) */
            case 5:  dac_sig = g_mc_debug.ib_a;             break;  /* phase B */
            case 6:  dac_sig = g_mc_debug.ic_a;             break;  /* phase C */
            case 7:  dac_sig = g_mc_debug.i_max_a;          break;  /* max |phase| */
            case 8:  dac_sig = g_mc_debug.i_arm_a;          break;  /* brushed armature */
            case 0:
            default: dac_sig = fabsf(g_mc_debug.iq_meas_a); break;  /* |iq| (default) */
        }
        MC_Dac_SetVolts(dac_sig * g_mc_inject.dac_scale_v_per_a);
    }
}

void MC_MotionLoop_1kHz(void)
{
    /* Stage B2: read the SSI encoder and update the state estimator. */
    /* Apply observer tuning + velocity-source selection from the OD (0x2500:3-6, GUI-settable + PERSIST). */
    s_est_cfg.obs_kp       = g_od.est_obs_kp;
    s_est_cfg.obs_ki       = g_od.est_obs_ki;
    s_est_cfg.obs_kv       = g_od.est_obs_kv;
    s_est_cfg.obs_filter_alpha = g_od.est_obs_filter_alpha;   /* observer output LPF, live-tunable (0x2500:7, ADR-003) */
    s_est_cfg.use_observer = (g_od.est_use_observer != 0u);

    /* Position feedback source. Tied to the backend for now (ADR-052, interim): the brushed axis uses the
       incremental quad on TIM2; the FOC axis uses the SSI. A clean per-board selector is the next step. */
    bool sample_ok;
    if (s_motor.backend_type == MC_MOTOR_BACKEND_BRUSHED_DC_HBRIDGE)
    {
        /* Incremental quad: continuous count -> rad (signed s_quad_rad_per_count). The estimator
           accumulates wrap_pi deltas, so it needs no single-turn anchor; absolute=false marks "no
           absolute position until homed" -- velocity is valid immediately, which closes the brushed
           velocity loop (ADR-052). */
        const int32_t cnt = MC_QuadEnc_Count();
        s_pos_sample.position_rad    = (float)cnt * s_quad_rad_per_count;
        s_pos_sample.raw_position    = (uint32_t)cnt;
        s_pos_sample.timestamp_ticks = 0u;
        s_pos_sample.valid           = true;
        s_pos_sample.absolute        = false;
        s_pos_sample.error           = false;
        s_pos_sample.warning         = false;
        sample_ok = true;
    }
    else
    {
        sample_ok = MC_SsiEncoder_ReadHardware(&s_enc, &s_enc_cfg, &s_pos_sample);
    }

    if (sample_ok)
    {
        MC_StateEstimator_Update(&s_est, &s_est_cfg, &s_pos_sample);

        /* Startup position anchor (ADR-037, hardened by ADR-038) -- single-turn ABSOLUTE encoders only
           (the incremental quad is already continuous and skips this, accumulating from the power-on
           count). The single-turn absolute encoder loses the turn count across a power cycle, so the
           continuous position must be anchored to the home-relative reading wrapped to the nearest turn.
           The original one-shot seed raced the persistent home load on a COLD boot: it could fire with
           s_home_offset_rad still 0, leaving continuous ~1 turn off everywhere except home (a soft reset
           hid it -- RAM kept the good anchor so the seed never re-ran). Hardened: while the drive has
           NEVER been enabled, re-anchor every cycle -- idempotent once correct, self-correcting if home
           loads late. s_pos_locked latches on the first enable so motion tracks true multi-turn. */
        if (s_pos_sample.absolute)
        {
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
    }

    /* One-shot startup position recall (ADR-067): for a non-back-drivable INCREMENTAL axis with the
       feature on, adopt the last stored position instead of demanding a re-home. Runs once, after the
       first good sample, while the drive is still off (before any motion). Absolute (SSI) axes
       self-locate and skip this. The incremental estimate accumulates from the power-on count (~0),
       so we reconstruct the home anchor to make position_actual read the stored value at the current
       count: home-relative = position - offset  =>  offset = position - stored_P. */
    if (sample_ok && !s_recall_applied)
    {
        s_recall_applied = true;
        if ((g_od.position_recall_enable != 0u) && !s_pos_sample.absolute &&
            MC_PosRecall_HasValidStored())
        {
            s_home_offset_rad          = s_est.mechanical.position_rad - MC_PosRecall_StoredPosition();
            g_mc_debug.home_offset_rad = s_home_offset_rad;
            s_homed                    = true;   /* recalled position substitutes for homing (ADR-067) */
            s_recall_used              = true;
            s_recall_was_moving        = false;
            s_recall_settle_ticks      = MC_RECALL_SETTLE_TICKS;  /* already settled at the recalled point */
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
        /* No valid persistent config -> treat as severe so the mode manager won't enable the operational
           drive (commissioning/align bypass this). Self-clears once a valid config loads or is saved (ADR-051). */
        fs.severe_active = s_oc_trip || !MC_PersistentStore_HasValid();
        MC_ModeManager_Update(&dc, &fs);
        const MC_DriveStatus_t ds = MC_ModeManager_GetStatus();

        s_eff_align_q = false; s_eff_hb_test = false;   /* default each cycle; the dq-test sets per-axis (ADR-046 ext) */
        s_jog_ref_vel = 0.0f;   /* default: no jog FF this tick unless the position-jog block sets it (ADR-073) */

        /* Homing sequencer (ADR-057, extracted ADR-068). home_command is a level -- 1 = run,
           0 = idle/reset; watch-inject and the dq-test preempt it. The module owns the phase/timers/
           status; the shared mech-zero anchor, slew limiter, OC-trip latch and persistence stay here
           and are driven by the output pulses. The arbiter's homing branch (below) applies the drive. */
        MC_HomingInput_t hin;
        hin.enable              = (g_od.home_command != 0u) && !g_mc_inject.inject_enable && !g_od.dq_test_enable;
        hin.clear               = (g_od.home_command == 0u);
        hin.mech_position_rad   = s_est.mechanical.position_rad;
        hin.mech_velocity_rad_s = s_est.mechanical.velocity_rad_per_s;
        hin.oc_trip             = s_oc_trip;
        hin.home_velocity_rad_s = g_od.home_velocity_rad_s;
        MC_HomingOutput_t hout;
        MC_Homing_Update(&s_homing, &hin, &hout);
        g_od.home_status = hout.status;
        if (hout.reset_slew)      { vel_slew_reset(s_est.mechanical.velocity_rad_per_s); }
        if (hout.capture_zero)    { s_home_offset_rad = s_est.mechanical.position_rad; g_mc_debug.home_offset_rad = s_home_offset_rad; }
        if (hout.consume_oc_trip) { s_oc_trip = false; }
        if (hout.completed)       { s_homed = true; params_save(); }   /* persist the zero captured at the stop */

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
            s_dq_test_ticks = 0u;
        }
        else if (g_od.dq_test_enable)
        {
            /* GUI-fired open-loop d-axis voltage step for plant ID (ADR-046): open-loop Vd at a fixed
               electrical angle, no current loop. The align path clamps Vd to +/-MC_C2_VD_MAX and the OC
               trip still protects. Pulse: hold Vd for dq_test_dwell_ms, then auto-return to 0 and disarm
               (dwell clamped to MC_DQ_TEST_MAX_MS as a backstop). This path overrides the remote enable,
               so the motor is live regardless of the axis_manager. */
            uint32_t dwell_ms = (g_od.dq_test_dwell_ms == 0u) ? 1u : (uint32_t)g_od.dq_test_dwell_ms;
            if (dwell_ms > MC_DQ_TEST_MAX_MS) { dwell_ms = MC_DQ_TEST_MAX_MS; }
            const bool dwell_done = (++s_dq_test_ticks > dwell_ms);  /* 1 tick = 1 ms (1 kHz arbitration) */
            if (dwell_done) { g_od.dq_test_enable = 0u; }            /* dwell elapsed -> pulse back to 0 */
            const bool  test_on = (!dwell_done) && (fabsf(g_od.dq_test_voltage_v) > 1e-3f);
            const float test_v  = test_on ? g_od.dq_test_voltage_v : 0.0f;
            if (g_od.dq_test_axis == 2u)        /* brushed_phase: open-loop H-bridge armature voltage */
            {
                s_eff_hb_test   = test_on;
                s_eff_hb_test_v = test_v;
                s_eff_align     = false;
            }
            else                                /* 0 = d-axis, 1 = q-axis: FOC open-loop SVPWM at the angle */
            {
                s_eff_align       = test_on;
                s_eff_align_v     = test_v;
                s_eff_align_angle = g_od.dq_test_angle_rad;
                s_eff_align_q     = (g_od.dq_test_axis == 1u);
            }
            s_eff_drive         = false;
            s_eff_torque_mode   = false;
            s_eff_position_mode = false;
            s_eff_halt          = false;
            s_eff_iq_cmd        = 0.0f;
            s_eff_id_cmd        = 0.0f;
            s_eff_vel_cmd       = 0.0f;
            g_od.statusword     = (uint16_t)((g_mc_debug.pwm_enabled ? MC_IF_SW_ENABLED : 0u)
                                           | (s_oc_trip ? MC_IF_SW_FAULT : 0u) | MC_IF_SW_READY);
        }
        else if (hout.active)
        {
            /* Homing owns this tick (ADR-057/068). The sequencer (above) advanced the phase/timers and
               emitted the drive command + any capture/persist side-effects; here we just set velocity
               mode and apply it. velocity_cmd is raw -> ramp through the slew limiter unless the module
               asked to bypass it (the one-tick zero at stop capture and the failed/done stops). */
            s_dq_test_ticks = 0u;
            s_eff_align       = false;
            s_eff_torque_mode = false;   /* velocity mode */
            s_eff_position_mode = false;
            s_eff_halt        = false;
            s_eff_iq_cmd      = 0.0f;
            s_eff_id_cmd      = 0.0f;
            s_eff_drive       = hout.want_drive;
            s_eff_vel_cmd     = hout.slew ? vel_slew_limit(hout.velocity_cmd) : hout.velocity_cmd;
            g_od.statusword = (uint16_t)((g_mc_debug.pwm_enabled ? MC_IF_SW_ENABLED : 0u)
                                       | (s_oc_trip ? MC_IF_SW_FAULT : 0u) | MC_IF_SW_READY);
        }
        else
        {
            /* Remote: the mode manager (OD/CiA-402) drives. Boot-safe (controlword 0 = Disabled). */
            s_dq_test_ticks = 0u;
            s_eff_align = false;
            s_eff_drive = ds.operation_enabled;
            const bool halt_rise = (ds.active_mode == MC_MODE_POSITION_HOLD) && !s_eff_halt;
            s_eff_halt = (ds.active_mode == MC_MODE_POSITION_HOLD);
            /* Jerk limiter (ADR-042): hold it at the live velocity unless PROFILE_VELOCITY is the active
               driven mode, so entering velocity mode is bump-free (the position cascade bypasses it). */
            if (!(s_eff_drive && ds.active_mode == MC_MODE_PROFILE_VELOCITY))
            {
                vel_slew_reset(s_est.mechanical.velocity_rad_per_s);
            }
            if (ds.active_mode == MC_MODE_TORQUE_CURRENT)
            {
                s_eff_torque_mode   = true;
                s_eff_position_mode = false;
                s_eff_iq_cmd        = (float)g_od.target_torque * MC_IF_CUR_SCALE;
                s_eff_id_cmd        = 0.0f;
            }
            else if (ds.active_mode == MC_MODE_PROFILE_VELOCITY)
            {
                s_eff_torque_mode = false;
                if (g_od.jog_position_mode == 0u)
                {
                    /* Direct velocity (default, ADR-042): the velocity loop tracks the ramped setpoint. */
                    s_eff_position_mode = false;
                    s_eff_vel_cmd       = vel_slew_limit(dc.target_velocity_rad_per_s);   /* cyclic velocity_setpoint, accel-ramp limited */
                }
                else
                {
                    /* Position-integrated jog (ADR-062): integrate the ramped velocity into the position
                       hold target and let the position cascade (D3) track it -- so following-error,
                       soft limits and a stiff hold all apply while jogging. D3 latches s_pos_hold_rad =
                       actual on entry (via s_pos_on); we integrate from there, leashing the reference to
                       the actual (can't run away from a stuck axis) and clamping it to the soft-limit band. */
                    s_eff_position_mode = true;
                    s_traj.active       = false;   /* streaming target, no planned move */
                    if (s_pos_on)
                    {
                        const float jog_vel  = vel_slew_limit(dc.target_velocity_rad_per_s);
                        const float p_act    = s_est.mechanical.position_rad - s_home_offset_rad;
                        const float ref_prev = s_pos_hold_rad;
                        s_pos_hold_rad += jog_vel * MC_MOTION_DT_S;
                        if      (s_pos_hold_rad > p_act + MC_JOG_LEASH_RAD) { s_pos_hold_rad = p_act + MC_JOG_LEASH_RAD; }
                        else if (s_pos_hold_rad < p_act - MC_JOG_LEASH_RAD) { s_pos_hold_rad = p_act - MC_JOG_LEASH_RAD; }
                        if (pos_limits_active())
                        {
                            if      (s_pos_hold_rad > g_od.pos_limit_hi_rad) { s_pos_hold_rad = g_od.pos_limit_hi_rad; }
                            else if (s_pos_hold_rad < g_od.pos_limit_lo_rad) { s_pos_hold_rad = g_od.pos_limit_lo_rad; }
                        }
                        /* Velocity feedforward for the jog (ADR-073): the FF is the ACTUAL per-tick advance of the
                           (leash/soft-limit-clamped) reference, not the raw jog_vel -- so when the leash or a limit
                           pins the reference, the advance (and thus the FF) collapses to 0 instead of over-driving.
                           Consumed by the position cascade's hold branch and scaled by s_vel_ff_gain (0x2200:4). */
                        s_jog_ref_vel = (s_pos_hold_rad - ref_prev) / MC_MOTION_DT_S;
                    }
                }
            }
            else if (ds.active_mode == MC_MODE_PROFILE_POSITION)
            {
                /* D3 (ADR-028): the position cascade below produces s_eff_vel_cmd; run it as a velocity
                   move. NEW_SETPOINT (rising edge, latched by the mode manager) starts a fresh plan. */
                s_eff_torque_mode   = false;
                s_eff_position_mode = true;
                /* Block position recalls until the incremental encoder has zeroed (ADR-057): an absolute
                   encoder (SSI) is always OK; an incremental one (quad) needs homing/zeroing first, else the
                   target would be relative to the meaningless power-on count. s_homed is not persisted. */
                if (ds.new_setpoint_latched && (s_pos_sample.absolute || s_homed))
                {
                    MC_TrajRequest_t req;
                    req.start.position_rad             = s_est.mechanical.position_rad - s_home_offset_rad;
                    req.start.velocity_rad_per_s       = 0.0f;
                    req.start.acceleration_rad_per_s2  = 0.0f;
                    {
                        float tgt = (float)g_od.target_position * MC_IF_POS_SCALE;
                        if (pos_limits_active())   /* clamp the target into the soft-limit band (ADR-040/043) */
                        {
                            const float lo = g_od.pos_limit_lo_rad, hi = g_od.pos_limit_hi_rad;
                            if      (tgt > hi) { tgt = hi; }
                            else if (tgt < lo) { tgt = lo; }
                        }
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
                    req.limits.max_jerk_rad_per_s3         = g_od.max_jerk_rad_s3;   /* S-curve planner (ADR-045); the trapezoid ignores it */
                    MC_PositionController_Reset(&s_pos_ctl);
                    /* Select the planner per 0x2600:9 (ADR-045). Both fill s_traj; the sampler is shared. */
                    (void)(g_od.traj_use_scurve ? MC_TrajScurve_Plan(&s_traj, &req)
                                                : MC_Trajectory_Start(&s_traj, &req));
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
            s_eff_align_q     = false;   /* electrical alignment is always d-axis */
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
        g_od.freq_sweep_current_hz = MC_FreqSweep_CurrentHz(&s_freq_sweep);  /* 0x2920:8 RO PDO (ADR-047) */
        g_od.freq_sweep_active     = MC_FreqSweep_Active(&s_freq_sweep) ? 1u : 0u;  /* 0x2920:9 RO */
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
        bool  at_cmd_target = true;   /* cleared for an abandoned/manual hold (post-jog) so on-target/on-shot isn't reported there (ADR-056) */
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
                /* No active plan (idle, or the plan was abandoned on a velocity-mode jog, ADR-056): hold the
                   latched position, but this is NOT a commanded target -- so don't report on-target/on-shot
                   here. The CMC's target was the recalled position, not a manual trim. */
                /* v_ff = s_jog_ref_vel is the jog reference velocity (Δref/dt) during a position-integrated
                   jog, and 0 for a genuine idle hold (the arbiter leaves it 0) -- so the jog gets velocity
                   feedforward (no following lag) while a static hold stays pure feedback (ADR-073). Scaled
                   by s_vel_ff_gain at the demand sum below, same as the trajectory FF. */
                p_dem = s_pos_hold_rad; v_ff = s_jog_ref_vel; a_ff = 0.0f; complete = true;
                at_cmd_target = false;
            }
        }

        const float vcorr = MC_PositionController_Update(&s_pos_ctl, &s_pos_cfg, p_dem, p_act);
        s_eff_vel_cmd     = (s_vel_ff_gain * v_ff) + vcorr;  /* velocity demand = FF-gain·FF + position correction (ADR-031) */
        s_accel_ff_rad_s2 = a_ff;             /* -> torque request inertia slot */

        const float perr = s_pos_ctl.position_error_rad;
        /* ON_TARGET / TARGET_REACHED tolerance = the position deadband (0x2200:5, ADR-071), so "at the
           shot" is reported over the same band the axis actually parks in under the deadband (ADR-076).
           Falls back to MC_POS_TARGET_WINDOW_RAD when the deadband is off (0) so the bit stays reachable. */
        const float twin = (s_pos_cfg.deadband_rad > 0.0f) ? s_pos_cfg.deadband_rad : MC_POS_TARGET_WINDOW_RAD;
        const bool reached = complete && at_cmd_target && (perr < twin) && (perr > -twin);
        if (reached) { g_od.statusword |= MC_IF_SW_TARGET_REACHED; }

        g_mc_debug.pos_demand_rad = p_dem;
        g_mc_debug.pos_actual_rad = p_act;
        g_mc_debug.pos_error_rad  = perr;
        g_mc_debug.target_reached = reached;
    }
    else
    {
        /* Leaving position mode (e.g. a velocity-mode joystick trim): abandon the plan so a later
           re-entry HOLDS THE CURRENT position, not the old target. Without this the completed trajectory
           stays active@target (MC_Trajectory_Evaluate keeps returning valid@target) and drives back on
           return; a fresh move still needs a new setpoint. (ADR-056) */
        if (s_pos_on) { s_traj.active = false; }
        s_pos_on          = false;
        s_accel_ff_rad_s2 = 0.0f;
        g_mc_debug.target_reached = false;
    }

    /* Stage D2: velocity cascade -> torque request -> iq, published to the fast loop. */
    const bool vel_active = s_eff_drive && !s_eff_torque_mode && !s_oc_trip;
    if (vel_active)
    {
        if (!s_vel_on) { MC_VelocityController_Reset(&s_vel); MC_Notch_Reset(&s_iq_notch); s_vel_on = true; }

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
        /* Soft position limits (ADR-043, refining ADR-040): taper the velocity demand so it lands AT a
           manually-set limit at zero speed instead of slamming/overshooting; never restrict motion AWAY
           from a limit, so you can always drive out of the zone. Decel budget = the envelope max_accel
           (0x2600:5); with it off (0) we fall back to a hard stop at the limit. Active only when homed. */
        if (pos_limits_active())
        {
            const float lo = g_od.pos_limit_lo_rad, hi = g_od.pos_limit_hi_rad;
            const float pos_rel = s_est.mechanical.position_rad - s_home_offset_rad;
            const float adec    = g_od.max_accel_rad_s2;        /* 0 => hard-stop fallback */
            if (vdem > 0.0f)                                     /* heading toward the hi limit */
            {
                const float d = hi - pos_rel;
                if (d <= 0.0f) { vdem = 0.0f; }                 /* at/past hi: stop further; away is untouched */
                else if (adec > 0.001f)
                {
                    const float v_allow = sqrtf(2.0f * adec * d);
                    if (vdem > v_allow) { vdem = v_allow; }      /* decel taper -> 0 at hi */
                }
            }
            else if (vdem < 0.0f)                                /* heading toward the lo limit */
            {
                const float d = pos_rel - lo;
                if (d <= 0.0f) { vdem = 0.0f; }
                else if (adec > 0.001f)
                {
                    const float v_allow = sqrtf(2.0f * adec * d);
                    if (vdem < -v_allow) { vdem = -v_allow; }
                }
            }
        }
        const float vact = s_est.mechanical.velocity_rad_per_s;   /* observer by default */

        /* Always actively hold while enabled (ADR-072, REQ-0016): the motor no longer autonomously
           releases holding current. Idle policy is the CMC's -- it commands op_mode = HOLD (keep
           regulating to zero velocity) or OFF (drive disabled via the controlword, which drops
           vel_active so the fast loop safe-offs the bridge). 0x2300:9 holding_enable is advisory only
           now; the old 1 s dwell-release logic (ADR-054) is gone. */
        {
            const float tcorr = MC_VelocityController_Update(&s_vel, &s_vel_cfg, vdem, vact);

            MC_CurrentRequestDebug_t crd;
            MC_MotorTorqueRequest_t treq =
                MC_CurrentRequest_Update(&s_torque_cfg, tcorr, s_accel_ff_rad_s2 /* trajectory accel FF (D3) */, vact, true, &crd);
            MC_FocCurrentCommand_t fcmd = MC_CurrentRequest_ToFocCommand(&s_torque_cfg, &treq);

            const float iq_notched = MC_Notch_Update(&s_iq_notch, fcmd.iq_a);   /* current-command notch (ADR-048) */
            s_iq_cmd_published = g_od.notch_enable ? iq_notched : fcmd.iq_a;
            /* Low-speed anti-stiction dither (ADR-066): add a zero-mean sine current, faded out
               as |velocity| -> threshold. AFTER the notch so it isn't filtered; the current limit
               downstream keeps it bounded. */
            MC_Dither_SetParams(g_od.dither_enable != 0u, g_od.dither_speed_threshold_rad_s,
                                g_od.dither_amplitude_a, g_od.dither_freq_hz);
            {
                const float dith = MC_Dither_Update(vact, MC_MOTION_DT_S);
                s_iq_cmd_published  += dith;
                g_od.dither_output_a = dith;
            }
            g_mc_debug.vel_demand_rad_s  = vdem;
            g_mc_debug.vel_torque_cmd_nm = treq.torque_nm;
            g_mc_debug.vel_iq_cmd_a      = fcmd.iq_a;
        }
    }
    else
    {
        s_vel_on = false;
        s_iq_cmd_published = 0.0f;
        g_od.dither_output_a = 0.0f;
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
        s_homed                    = true;   /* manual zero also un-gates position recalls (ADR-057) */
        params_save();
    }

    /* Set mechanical zero to a COMMANDED value (SET_MECH_ZERO_AT): the tool supplies mech_zero_set_rad
       -- e.g. the midpoint of two captured travel extremes -- so the home is centred without driving
       the axis there. Same absolute (multi-turn) frame as the capture. Auto-saved. (ADR-022) */
    if (s_set_zero_at_pending)
    {
        s_set_zero_at_pending      = false;
        s_home_offset_rad          = g_od.mech_zero_set_rad;
        g_mc_debug.home_offset_rad = s_home_offset_rad;
        s_homed                    = true;
        params_save();
    }

    od_mirror_live();   /* publish live state into the OD store */
}

/* Position-recall journal service (ADR-067), slow/supervisory context. Incremental axes only, and
   only once homed (an un-homed position is meaningless and must never be journalled as VALID). On
   the MOVING rising edge the stored position is invalidated (a mid-move power loss then reverts to
   NOT_HOMED); after MOVING stays clear for the settle dwell the settled position is stored VALID
   (store-on-change). Flash writes here are in bank2 (POS_RECALL, pg124/125) -- read-while-write vs
   the bank1-resident hot ISR code, the same basis the config store relies on -- so they are NOT
   gated on the power stage being off (the invalidate write inherently fires while the drive runs). */
static void pos_recall_service_slow(void)
{
    if ((g_od.position_recall_enable == 0u) || s_pos_sample.absolute || !s_homed)
    {
        s_recall_was_moving   = false;
        s_recall_settle_ticks = 0u;
        return;
    }

    const float v = s_est.mechanical.velocity_rad_per_s;
    const bool  moving = (v > MC_RECALL_MOVE_EPS) || (v < -MC_RECALL_MOVE_EPS);

    if (moving && !s_recall_was_moving)
    {
        MC_PosRecall_MarkMoving();          /* motion started -> stored position is now stale */
        s_recall_settle_ticks = 0u;
    }
    else if (!moving && (s_recall_settle_ticks < MC_RECALL_SETTLE_TICKS))
    {
        s_recall_settle_ticks++;
        if (s_recall_settle_ticks == MC_RECALL_SETTLE_TICKS)
        {
            const float p = s_est.mechanical.position_rad - s_home_offset_rad;
            /* Re-store on a real position change, or whenever the latest record is not VALID
               (e.g. we invalidated at move start but ended near the same place). */
            if (!MC_PosRecall_HasValidStored() ||
                (fabsf(p - MC_PosRecall_StoredPosition()) > MC_RECALL_STORE_EPS))
            {
                MC_PosRecall_Store(p);
            }
        }
    }
    s_recall_was_moving = moving;
}

/* Slow-loop OD-command dispatch (ADR-068 phase 4): flat routing of OD command words to latched
   requests -- no control math, no motion. Each handler consumes its command field and latches a
   request the appropriate loop/service acts on. Extracted verbatim from MC_SlowLoop_10_100Hz so the
   loop body reads as a sequence of services; the factory-reset / save magics are wired unchanged. */
static void od_commands_service_slow(void)
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
    if (g_od.cal_command == MC_IF_CAL_SET_MECH_ZERO_AT)   /* set mech home to mech_zero_set_rad (0x2700:10, ADR-022) */
    {
        s_set_zero_at_pending = true;
        g_od.cal_status  = MC_IF_CAL_SET_MECH_ZERO_AT;
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
}

/* Persistence service (ADR-010/068 phase 3): commit pending saves + a requested factory reset, only
   with the power stage off so a flash erase/program can't disturb an active drive. */
static void persistence_service_slow(void)
{
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
    g_mc_debug.store_save_pending  = MC_PersistentStore_SavePending();
    g_mc_debug.store_blob_truncated = MC_Od_PersistTruncated();   /* recurrence guard (ADR-070) */
}

/* Winding thermal model service (ADR-065/068 phase 3): advance the I²t estimate at the slow rate
   (100 Hz) from the measured motor-current magnitude, then mirror utilisation + derate to the OD
   (0x2100:4/5). Backend-aware current: |i_arm| for brushed, sqrt(id^2+iq^2) for FOC. Must run before
   od_apply_gains() so the derated current limit is applied this tick. */
static void thermal_service_slow(void)
{
    const float i_thermal = (g_od.motor_backend_sel == 1u)
        ? fabsf(g_od.tlm_i_arm_a)
        : sqrtf(g_od.tlm_id_meas_a * g_od.tlm_id_meas_a + g_od.tlm_iq_meas_a * g_od.tlm_iq_meas_a);
    MC_Thermal_SetParams(g_od.thermal_enable != 0u, g_od.thermal_i_cont_a, g_od.thermal_tau_s,
                         g_od.thermal_derate_start);
    MC_Thermal_Update(i_thermal, MC_MOTION_DT_S * 10.0f);   /* slow loop = 1 kHz / 10 = 100 Hz */
    g_od.thermal_utilisation   = MC_Thermal_Utilisation();
    g_od.thermal_derate_factor = MC_Thermal_DerateFactor();
}

/* Inter-MCU command dead-man (ADR-068 phase 3), REMOTE mode only: a stale cyclic-command stream
   zeroes the remote velocity demand (the OD target). Skipped in commissioning so the watch-window
   command is never clobbered. Full quick-stop is the fault manager's job (E2). */
static void command_deadman_service_slow(void)
{
    if (!g_mc_inject.inject_enable && MC_Comms_CommandTimedOut())
    {
        g_od.target_velocity = 0;
    }
}

void MC_SlowLoop_10_100Hz(void)
{
    /* Slow/supervisory context (100 Hz). A thin dispatcher: run each service in order, then apply
       OD-written gains at this safe update point. Ordering matters -- thermal_service_slow() derates
       the current limit that od_apply_gains() then pushes to the live controllers this tick. */
    MC_BootMeta_Tick();               /* dual-bootloader healthy-window STAY-flag clear (REQ-0015) */
    od_commands_service_slow();       /* route OD command words to latched requests (ADR-068 phase 4) */
    persistence_service_slow();       /* commit saves / factory reset when the drive is off (ADR-010) */
    pos_recall_service_slow();        /* journal the last position / invalidate on move start (ADR-067) */
    thermal_service_slow();           /* advance the I²t model + mirror utilisation/derate (ADR-065) */
    od_apply_gains();                 /* apply OD-written gains to the live controllers (safe point) */
    command_deadman_service_slow();   /* zero a stale remote velocity demand (REMOTE only) */
}
