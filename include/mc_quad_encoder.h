#ifndef MC_QUAD_ENCODER_H
#define MC_QUAD_ENCODER_H
#include "mc_types.h"

/** @file mc_quad_encoder.h
 *  @brief Quadrature-encoder count accessor (TIM2 in TI12 4x encoder mode, ADR-050).
 *
 *  HAL/register access lives in the STM32 boundary (mc_quad_encoder_stm32g474.c). The scheduler reads
 *  the count each medium cycle and mirrors it to OD 0x2510:4 for a manual diagnostic read from the PC
 *  tool. The timer free-runs as a 32-bit counter; the count is reported signed (negative = reverse for
 *  the typical near-zero working range). Start (HAL_TIM_Encoder_Start) is done at boot in main.c.
 *  This is a raw count for bring-up; folding it into the position-feedback interface is a later step.
 */
int32_t MC_QuadEnc_Count(void);

#endif /* MC_QUAD_ENCODER_H */
