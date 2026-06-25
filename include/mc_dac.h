#ifndef MC_DAC_H
#define MC_DAC_H
#include "mc_types.h"

/** @file mc_dac.h
 *  @brief Debug analog output on the board DAC (DAC1_OUT1 / PA4) for scoping a live signal.
 *  @ingroup mc_debug
 *
 *  Bring-up aid (ADR-005): mirror an internal value (e.g. the measured current) onto a pin so it
 *  can be viewed on a scope at the full loop rate. HAL-free interface; the STM32 implementation is
 *  in the boundary (mc_dac_stm32g474.c).
 */

/** @brief Start DAC1 channel 1 (PA4). Call once at init (the CubeMX MX_DAC1_Init only configures it). */
void MC_Dac_Init(void);

/** @brief Drive the DAC output to @p volts, clamped to [0, Vref]. 12-bit, software-updated. */
void MC_Dac_SetVolts(float volts);

#endif /* MC_DAC_H */
