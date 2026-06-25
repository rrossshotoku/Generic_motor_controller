#ifndef MC_HBRIDGE_H
#define MC_HBRIDGE_H
#include "mc_types.h"

/** @file mc_hbridge.h
 *  @brief Brushed-DC H-bridge modulator (locked anti-phase). See ADR-039.
 *  @ingroup mc_backend
 *
 *  Maps a signed motor (armature) voltage command to the two driven-leg duties using
 *  **locked anti-phase**: both legs PWM in anti-phase about 50 %, so 50 % = 0 V and the output
 *  is linear through zero with no dead-band:
 *
 *      V_motor = Vbus * (2*duty_a - 1),   duty_b = 1 - duty_a
 *
 *  The third inverter leg is unused (disconnected from the motor) -- its duty is left at 0.5 and
 *  its outputs are disabled in the PWM boundary. HAL-free and host-testable; the brushed fast-loop
 *  path calls this exactly where the FOC path calls MC_Foc_Update (both yield an MC_PwmDuty_t).
 */

/** @brief Locked anti-phase modulator configuration. */
typedef struct
{
    float vbus_v;          /**< DC bus voltage [V] (must be > 0). */
    float max_modulation;  /**< |m| ceiling in (0,1], keeps the legs off the rails (e.g. 0.95). */
} MC_HBridgeConfig_t;

/**
 * @brief Locked anti-phase modulation: signed armature voltage -> leg-A/B duties (leg C unused).
 *
 * @param v_cmd_v Signed motor voltage command [V] (+ = the wiring's "forward"; see phase_current_signs).
 * @param cfg     Bus voltage + modulation ceiling (non-NULL; vbus_v guarded to >= 1 V internally).
 * @return Duties for legs A and B (anti-phase about 0.5, clamped by @p max_modulation); duty_c = 0.5
 *         (unused leg), enable = true. Output duties are always within [0,1].
 *
 * @note `max_modulation` should leave headroom for the dead-time at full output -- e.g. set it to
 *       about `1 - 2*(dead_time/PWM_period)`; at zero output (50 % duty) the dead-time never clips.
 */
MC_PwmDuty_t MC_HBridge_LockedAntiphase(float v_cmd_v, const MC_HBridgeConfig_t *cfg);

#endif /* MC_HBRIDGE_H */
