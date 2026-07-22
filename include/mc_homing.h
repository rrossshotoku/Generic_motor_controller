#ifndef MC_HOMING_H
#define MC_HOMING_H
#include "mc_types.h"

/** @file mc_homing.h
 *  @brief Home-to-hard-stop sequencer (ADR-057), extracted from the scheduler (ADR-068).
 *  @ingroup mc_motion
 *
 *  A pure, HAL-free state machine: given the axis motion + the home command each medium tick, it
 *  drives an approach toward a hard end stop, detects the stop (no-movement dwell OR an over-current
 *  trip), captures the datum there, backs off clear of the stop, and finishes. It owns only the
 *  sequencing (phase, timers, status); the scheduler owns the shared mechanical-zero anchor, the
 *  velocity slew limiter, the OC-trip latch, and persistence — the module *signals* those via the
 *  output struct rather than touching them, so behaviour matches the former inlined version exactly.
 *
 *  Call `MC_Homing_Update` once per medium (1 kHz) tick and apply the returned command:
 *   - mirror `status` to the OD (0x2700:9);
 *   - if `active`, the command arbiter takes the homing branch: set velocity mode, `s_eff_drive =
 *     want_drive`, `s_eff_vel_cmd = slew ? vel_slew_limit(velocity_cmd) : velocity_cmd`;
 *   - honour `reset_slew` / `capture_zero` / `consume_oc_trip` / `completed` side-effects.
 */

/** @brief Homing sequencer state. Treat as opaque; zero-initialised by MC_Homing_Init. */
typedef struct {
    uint8_t  status;         /**< MC_IF_HOME_* (IDLE/RUNNING/DONE/FAILED). */
    bool     backing_off;    /**< false = approach phase, true = back-off phase. */
    bool     moved;          /**< armed once the axis has moved (so ramp-up isn't read as the stop). */
    uint32_t still_ticks;    /**< consecutive not-moving ticks in the approach. */
    uint32_t total_ticks;    /**< elapsed ticks since start, for the safety timeout. */
    uint32_t backoff_ticks;  /**< elapsed ticks in the back-off phase. */
} MC_Homing_t;

/** @brief Inputs sampled by the caller each medium tick. */
typedef struct {
    bool  enable;               /**< home_command != 0 AND not preempted (inject / dq-test). */
    bool  clear;                /**< home_command == 0 -> clear a latched DONE/FAILED back to IDLE. */
    float mech_position_rad;    /**< current mechanical position (continuous). */
    float mech_velocity_rad_s;  /**< current mechanical velocity. */
    bool  oc_trip;              /**< latched over-current trip (a hard-stop detector). */
    float home_velocity_rad_s;  /**< approach velocity [rad/s]; sign = direction (0x2700:6). */
} MC_HomingInput_t;

/** @brief Command produced each tick. Fields with side-effects are one-shot pulses. */
typedef struct {
    uint8_t status;          /**< MC_IF_HOME_* -> mirror to g_od.home_status. */
    bool    active;          /**< status == RUNNING: the arbiter should take the homing branch. */
    bool    want_drive;      /**< s_eff_drive for this tick. */
    float   velocity_cmd;    /**< target velocity; raw (see slew). */
    bool    slew;            /**< true: caller applies vel_slew_limit(velocity_cmd); false: use as-is. */
    bool    reset_slew;      /**< pulse: caller does vel_slew_reset(current velocity) this tick. */
    bool    capture_zero;    /**< pulse: caller captures the mechanical zero at the current position. */
    bool    consume_oc_trip; /**< pulse: caller clears its latched OC trip. */
    bool    completed;       /**< pulse: homing finished OK -> caller sets homed + persists. */
} MC_HomingOutput_t;

/** @brief Reset the sequencer to IDLE. */
void MC_Homing_Init(MC_Homing_t *h);

/** @brief Advance the sequencer one medium tick and emit the command for this tick. */
void MC_Homing_Update(MC_Homing_t *h, const MC_HomingInput_t *in, MC_HomingOutput_t *out);

#endif /* MC_HOMING_H */
