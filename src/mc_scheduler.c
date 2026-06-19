#include "mc_scheduler.h"
#include "mc_debug.h"
#include "mc_current_sense.h"

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

void MC_Framework_Init(void)
{
    MC_Debug_Init();
    MC_CurrentSense_Init(&s_cs);
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
}

void MC_MotionLoop_1kHz(void)
{
    /* Stage A1: no motion work yet. */
}

void MC_SlowLoop_10_100Hz(void)
{
    /* Stage A1: no supervisory work yet. */
}
