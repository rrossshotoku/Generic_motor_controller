#ifndef MC_DEBUG_H
#define MC_DEBUG_H
#include "mc_types.h"

/** @file mc_debug.h
 *  @brief Bring-up observability: live watch-window mirror + command-injection harness.
 *  @ingroup mc_core
 *
 *  Per ADR-005, early bring-up is done on-target over SWD using the debugger's live watch
 *  window rather than a host test suite. Two global, `volatile` structures form the
 *  interface:
 *   - @ref g_mc_debug  : a snapshot of framework internals (loop cadence, timing,
 *     power-stage state) to add to the watch window for observation.
 *   - @ref g_mc_inject : command/inject fields written from the watch window to exercise
 *     modules during bring-up. Nothing here drives the plant unless @c inject_enable is set.
 *
 *  Loop timing uses the Cortex-M DWT cycle counter (at 170 MHz, 1 cycle ≈ 5.88 ns; the
 *  20 kHz fast period is ≈ 8500 cycles, the 1 kHz medium period ≈ 170000 cycles).
 */

/** @brief Live snapshot of framework internals for the watch window (written by the scheduler). */
typedef struct
{
    uint32_t fast_count;           /**< 20 kHz fast-loop iteration counter. */
    uint32_t medium_count;         /**< 1 kHz medium-loop iteration counter. */
    uint32_t slow_count;           /**< 100 Hz slow-loop iteration counter. */

    uint32_t fast_cycles;          /**< Last fast-loop body duration [CPU cycles]. */
    uint32_t fast_cycles_max;      /**< Worst-case fast-loop body duration [CPU cycles]. */
    uint32_t medium_cycles;        /**< Last medium-loop body duration [CPU cycles]. */
    uint32_t medium_cycles_max;    /**< Worst-case medium-loop body duration [CPU cycles]. */
    uint32_t slow_cycles;          /**< Last slow-loop body duration [CPU cycles]. */

    uint32_t fast_period_cycles;   /**< Measured interval between fast-loop entries [cycles]. */
    uint32_t medium_period_cycles; /**< Measured interval between medium-loop entries [cycles]. */

    uint32_t fast_overrun_count;   /**< Times the fast body exceeded its period budget. */
    uint32_t slow_missed_count;    /**< Times a slow tick was raised before the previous serviced. */

    bool pwm_enabled;              /**< Power-stage mirror; false = safe-off. */

    /* --- Current sense (Stage B1) --- */
    float    ia_a;                 /**< Phase A current [A]. */
    float    ib_a;                 /**< Phase B current [A] (derived). */
    float    ic_a;                 /**< Phase C current [A]. */
    uint16_t ia_raw;              /**< Phase A raw ADC counts. */
    uint16_t ic_raw;              /**< Phase C raw ADC counts. */
    float    ia_offset;           /**< Phase A zero-current offset [counts]. */
    float    ic_offset;           /**< Phase C zero-current offset [counts]. */
    bool     current_valid;       /**< Last current read valid (no ADC overrun). */
    bool     current_calibrated;  /**< Offsets have been calibrated. */

    /* --- Feedback / state estimator (Stage B2) --- */
    uint32_t enc_raw;             /**< Raw 21-bit SSI position [counts]. */
    float    mech_position_rad;   /**< Continuous (multi-turn) mechanical position [rad]. */
    float    mech_velocity_rad_s; /**< Mechanical velocity (active source) [rad/s]. */
    float    vel_finite_diff;     /**< Finite-difference velocity [rad/s]. */
    float    vel_observer;        /**< Position-tracking observer velocity [rad/s]. */
    float    elec_angle_rad;      /**< Electrical angle [0,2pi) [rad]. */
    bool     enc_valid;           /**< Last encoder read valid. */

    /* --- Open-loop drive / alignment (Stage C2) --- */
    float    vd_applied_v;        /**< Applied d-axis voltage [V]. */
    float    i_max_a;             /**< Max |phase current| this cycle [A]. */
    float    elec_offset_rad;     /**< Captured electrical offset [rad]. */
    bool     overcurrent_trip;    /**< Latched over-current trip (forces safe-off). */

    /* --- FOC current loop (Stage D1) --- */
    float    id_meas_a;           /**< Measured d-axis current [A]. */
    float    iq_meas_a;           /**< Measured q-axis current [A]. */
    float    vd_v;                /**< FOC d-axis voltage output [V]. */
    float    vq_v;                /**< FOC q-axis voltage output [V]. */
    bool     voltage_saturated;   /**< FOC voltage vector hit the limit. */
} MC_Debug_t;

/** @brief Command/inject fields written from the watch window during bring-up.
 *  All effects are gated by @c inject_enable so the harness cannot drive hardware by accident. */
typedef struct
{
    bool  inject_enable;        /**< Master gate: when false, inject fields have no effect. */
    bool  request_offset_cal;   /**< Bench: average N samples at zero current to set ADC offsets (PWM off). */
    bool  use_finite_diff_velocity; /**< Live: true = finite-diff velocity; false = observer (default). */
    float obs_kp;               /**< Live observer proportional gain (not gated; no drive). */
    float obs_ki;               /**< Live observer integral gain. */
    float obs_kv;               /**< Live observer velocity-damping gain. */

    /* --- Open-loop drive / alignment (Stage C2; all DRIVE gated by inject_enable above) --- */
    float align_voltage_v;       /**< Commanded d-axis voltage Vd [V] (clamped <= 3 V). */
    float align_angle_rad;       /**< Commanded electrical angle [rad] (0 aligns d-axis to phase A). */
    float vbus_v;                /**< Supply/bus voltage used for the duty calc [V]. */
    float current_limit_a;       /**< Over-current trip threshold [A]. */
    bool  request_align_capture; /**< Capture the electrical offset at the held rotor position. */
    bool  clear_fault;           /**< Clear the latched over-current trip. */

    /* --- FOC current loop (Stage D1; gated by inject_enable) --- */
    bool  foc_enable;            /**< Select the closed FOC current loop (vs open-loop drive). */
    float iq_cmd_a;              /**< q-axis (torque) current command [A]. */
    float id_cmd_a;              /**< d-axis current command [A] (usually 0). */
    float scratch_f;            /**< General-purpose value for early per-module bring-up. */
} MC_Inject_t;

/** @brief Live framework snapshot (add to the debugger watch window). */
extern volatile MC_Debug_t  g_mc_debug;
/** @brief Command-injection block (write from the debugger watch window). */
extern volatile MC_Inject_t g_mc_inject;

/** @brief Initialise the debug harness and enable the DWT cycle counter (target build). */
void MC_Debug_Init(void);

/** @brief Free-running CPU cycle count (DWT->CYCCNT) for loop-timing measurement. */
uint32_t MC_Debug_Cycles(void);

#endif /* MC_DEBUG_H */
