#include "mc_current_sense.h"
#include "mc_board_config.h"
#include "mc_debug.h"     /* MC_Debug_Cycles() for the sample timestamp */
#include "adc.h"          /* HAL: hadc1/hadc2, dual-mode common-register read (boundary) */

/** @file mc_current_sense_stm32g474.c
 *  @brief STM32G474 three-shunt current-sense boundary (ADC1+ADC2 dual regular-simultaneous).
 *         The phase currents are sampled at the PWM peak via the TIM1-TRGO trigger; this
 *         module reads the dual common data register and converts to amps. See ADR-007.
 */

/* Offset-calibration accumulator (single-context: the fast loop). */
static bool     s_cal_active;
static uint16_t s_cal_target;
static uint16_t s_cal_n;
static uint32_t s_cal_sum_a;
static uint32_t s_cal_sum_c;

/* Read the latest simultaneous pair: CDR low16 = ADC1 master (phase A),
   high16 = ADC2 slave (phase C). */
static void read_raw(uint16_t *raw_a, uint16_t *raw_c)
{
    uint32_t cdr = HAL_ADCEx_MultiModeGetValue(&hadc1);
    *raw_a = (uint16_t)(cdr & 0xFFFFu);
    *raw_c = (uint16_t)((cdr >> 16) & 0xFFFFu);
}

void MC_CurrentSense_Init(MC_CurrentSense_t *cs)
{
    MC_BoardConfig_t board;
    MC_BoardConfig_LoadProfile0(&board);

    const float apc  = MC_CurrentSense_AmpsPerCount(&board.current_sense);
    const float zero = (float)MC_CurrentSense_ZeroCount(&board.current_sense);

    cs->amps_per_count_a = (float)board.current_sense.phase_sign[0] * apc;
    cs->amps_per_count_b = (float)board.current_sense.phase_sign[1] * apc;
    cs->amps_per_count_c = (float)board.current_sense.phase_sign[2] * apc;
    cs->offset_a_counts  = zero;   /* nominal until calibrated */
    cs->offset_b_counts  = zero;
    cs->offset_c_counts  = zero;
    cs->last_raw_a       = 0u;
    cs->last_raw_c       = 0u;
    cs->calibrated       = false;

    s_cal_active = false;
}

bool MC_CurrentSense_CalibrateOffsets(MC_CurrentSense_t *cs, uint16_t sample_count)
{
    if (!s_cal_active)
    {
        s_cal_active = true;
        s_cal_target = (sample_count == 0u) ? 1u : sample_count;
        s_cal_n      = 0u;
        s_cal_sum_a  = 0u;
        s_cal_sum_c  = 0u;
    }

    uint16_t raw_a, raw_c;
    read_raw(&raw_a, &raw_c);
    cs->last_raw_a = raw_a;
    cs->last_raw_c = raw_c;
    s_cal_sum_a += raw_a;
    s_cal_sum_c += raw_c;
    s_cal_n++;

    if (s_cal_n >= s_cal_target)
    {
        cs->offset_a_counts = (float)s_cal_sum_a / (float)s_cal_n;
        cs->offset_c_counts = (float)s_cal_sum_c / (float)s_cal_n;
        cs->offset_b_counts = 0.0f;   /* phase B is derived */
        cs->calibrated      = true;
        s_cal_active        = false;
        return true;
    }
    return false;
}

bool MC_CurrentSense_ReadFast(MC_CurrentSense_t *cs, MC_PhaseCurrents_t *currents)
{
    bool valid = true;

    /* Stale-data guard: an ADC2 (slave) overrun invalidates the simultaneous pair. */
    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR))
    {
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
        valid = false;
    }

    uint16_t raw_a, raw_c;
    read_raw(&raw_a, &raw_c);
    cs->last_raw_a = raw_a;
    cs->last_raw_c = raw_c;

    const float ia = ((float)raw_a - cs->offset_a_counts) * cs->amps_per_count_a;
    const float ic = ((float)raw_c - cs->offset_c_counts) * cs->amps_per_count_c;
    const float ib = -(ia + ic);   /* Kirchhoff: phase B derived from A and C */

    currents->ia_a            = ia;
    currents->ib_a            = ib;
    currents->ic_a            = ic;
    currents->valid           = valid;
    currents->timestamp_ticks = MC_Debug_Cycles();
    return valid;
}
