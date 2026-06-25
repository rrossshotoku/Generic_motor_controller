#include "mc_dac.h"
#include "dac.h"     /* HAL: hdac1 (boundary) */

/** @file mc_dac_stm32g474.c
 *  @brief STM32G474 debug-DAC boundary: DAC1_OUT1 on PA4, 12-bit, software-updated (no trigger).
 *         Started here because CubeMX's MX_DAC1_Init only configures the channel. See mc_dac.h.
 */

#define MC_DAC_VREF_V    3.3f
#define MC_DAC_FULLSCALE 4095.0f

void MC_Dac_Init(void)
{
    (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void MC_Dac_SetVolts(float volts)
{
    float v = volts;
    if (v < 0.0f)          { v = 0.0f; }
    if (v > MC_DAC_VREF_V) { v = MC_DAC_VREF_V; }

    uint32_t code = (uint32_t)((v / MC_DAC_VREF_V) * MC_DAC_FULLSCALE + 0.5f);
    if (code > 4095u) { code = 4095u; }

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
}
