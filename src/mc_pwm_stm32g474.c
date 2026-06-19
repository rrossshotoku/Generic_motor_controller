#include "mc_pwm.h"
#include "tim.h"   /* HAL: htim1 (TIM1 advanced timer) -- boundary module */

/** @file mc_pwm_stm32g474.c
 *  @brief STM32G474 3-phase PWM backend on TIM1. See ADR-009.
 *
 *  The TIM1 time base runs continuously (started in main) to generate the ADC trigger.
 *  Output drive is gated by the main output enable (MOE): MC_Pwm_Start arms the channels and
 *  sets MOE; MC_Pwm_ForceSafeOff clears MOE for an immediate, ISR-safe safe-off. Duties are
 *  written as compare values (centre-aligned).
 */

static uint32_t s_period = 4250u;   /* TIM1 ARR (centre-aligned) */

void MC_Pwm_Start(void)
{
    s_period = htim1.Init.Period;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void MC_Pwm_Stop(void)
{
    MC_Pwm_ForceSafeOff();
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void MC_Pwm_ForceSafeOff(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);   /* immediate: outputs to inactive (idle low) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s_period / 2u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, s_period / 2u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s_period / 2u);
}

void MC_Pwm_SetDutyFast(const MC_PwmDuty_t *duty)
{
    float fa = duty->duty_a;
    float fb = duty->duty_b;
    float fc = duty->duty_c;
    if (fa < 0.0f) { fa = 0.0f; } else if (fa > 1.0f) { fa = 1.0f; }
    if (fb < 0.0f) { fb = 0.0f; } else if (fb > 1.0f) { fb = 1.0f; }
    if (fc < 0.0f) { fc = 0.0f; } else if (fc > 1.0f) { fc = 1.0f; }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(fa * (float)s_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(fb * (float)s_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(fc * (float)s_period));
}
