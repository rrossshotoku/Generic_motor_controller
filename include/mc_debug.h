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
} MC_Debug_t;

/** @brief Command/inject fields written from the watch window during bring-up.
 *  All effects are gated by @c inject_enable so the harness cannot drive hardware by accident. */
typedef struct
{
    bool  inject_enable;        /**< Master gate: when false, inject fields have no effect. */
    bool  request_pwm_safe_off; /**< Set from the watch window to force the power stage to safe-off. */
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
