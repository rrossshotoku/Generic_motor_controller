#ifndef MC_DITHER_H
#define MC_DITHER_H
#include "mc_types.h"

/** @file mc_dither.h
 *  @brief Low-speed anti-stiction current dither (ADR-066).
 *  @ingroup mc_control
 *
 *  A zero-mean sine current, faded out as |velocity| approaches a threshold, added to the
 *  velocity/position-loop current command (after the notch) in the medium loop. Keeps the
 *  mechanism out of stiction at low speed. HAL-free / host-compilable.
 */

/** @brief Reset to the safe disabled state (no injection, phase 0). */
void  MC_Dither_Init(void);

/** @brief Apply OD config (0x2320:1..4). Cheap; safe to call every medium tick. */
void  MC_Dither_SetParams(bool enable, float speed_threshold_rad_s, float amplitude_a, float freq_hz);

/** @brief Advance the oscillator by @p dt_s and return the dither current [A] to ADD to the
 *  current command. 0 when disabled/unconfigured or |velocity| >= threshold. Zero-mean. */
float MC_Dither_Update(float velocity_rad_per_s, float dt_s);

#endif /* MC_DITHER_H */
