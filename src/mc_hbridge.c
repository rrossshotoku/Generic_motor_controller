#include "mc_hbridge.h"

/** @file mc_hbridge.c
 *  @brief Locked anti-phase H-bridge modulator for the brushed-DC backend. See ADR-039.
 *
 *  Pure, HAL-free. V_motor = Vbus*(2*duty_a - 1) with duty_b = 1 - duty_a, so the command maps
 *  linearly through zero (m = 0 -> both legs 50 % -> 0 V). The unused leg sits at 0.5 and is
 *  output-disabled in the PWM boundary; since it is also disconnected from the motor, no current
 *  flows through it regardless.
 */

MC_PwmDuty_t MC_HBridge_LockedAntiphase(float v_cmd_v, const MC_HBridgeConfig_t *cfg)
{
    /* Normalised command: m = +/-1 maps to +/-Vbus across the motor. Guard a zero/garbage bus. */
    const float vbus = (cfg->vbus_v > 1.0f) ? cfg->vbus_v : 1.0f;
    float m = v_cmd_v / vbus;

    /* Clamp |m| to the modulation ceiling (keeps the legs off the dead-time-limited duty rails at
       full output; at zero output the legs sit at 0.5 and never get near a rail). */
    float mmax = cfg->max_modulation;
    if ((mmax <= 0.0f) || (mmax > 1.0f)) { mmax = 1.0f; }
    if (m >  mmax) { m =  mmax; }
    if (m < -mmax) { m = -mmax; }

    /* Locked anti-phase about 50 %. duty_a in [0.5 - 0.5*mmax, 0.5 + 0.5*mmax] -> always within [0,1]. */
    MC_PwmDuty_t duty;
    duty.duty_a = 0.5f + 0.5f * m;
    duty.duty_b = 0.5f - 0.5f * m;   /* = 1 - duty_a (anti-phase) */
    duty.duty_c = 0.5f;              /* unused leg: disconnected + output-disabled in the PWM boundary */
    duty.enable = true;
    return duty;
}
