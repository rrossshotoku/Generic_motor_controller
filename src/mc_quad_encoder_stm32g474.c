#include "mc_quad_encoder.h"
#include "tim.h"   /* htim2 -- CubeMX TIM2 in TI12 encoder mode */

/** @file mc_quad_encoder_stm32g474.c
 *  @brief STM32 boundary for the TIM2 quadrature count (ADR-050). The timer free-runs 32-bit in 4x
 *  (TI12) encoder mode; the int32 cast gives a signed count around the power-on zero.
 */
int32_t MC_QuadEnc_Count(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}
