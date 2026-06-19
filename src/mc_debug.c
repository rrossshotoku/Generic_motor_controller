#include "mc_debug.h"
#include "stm32g4xx.h"   /* CMSIS core (DWT / CoreDebug cycle counter) — target build only. */

/** @file mc_debug.c
 *  @brief Bring-up debug-mirror/inject globals and DWT cycle-counter access.
 */

volatile MC_Debug_t  g_mc_debug;
volatile MC_Inject_t g_mc_inject;

void MC_Debug_Init(void)
{
    /* Globals are zero-initialised by C startup; here we only enable the DWT cycle
       counter used for loop-timing measurement. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t MC_Debug_Cycles(void)
{
    return DWT->CYCCNT;
}
