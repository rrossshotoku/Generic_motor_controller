#ifndef MC_SIGNAL_GEN_H
#define MC_SIGNAL_GEN_H
#include "mc_types.h"

/** @file mc_signal_gen.h
 *  @brief Reference test-signal generator for loop tuning (ADR-030). Pure / HAL-free / host-testable.
 *
 *  Produces a self-contained reference profile from rest: ramp to a (signed) peak amplitude at a ramp
 *  rate, dwell ("coast") for a hold time, then ramp back to 0. `rate <= 0` makes the edges
 *  instantaneous (a step). With `continuous`, after returning to 0 it pauses (`pause_s`) then flips the
 *  peak sign and repeats (an alternating ping-pong train) until stopped. `max_accel > 0` makes each ramp
 *  acceleration-limited (trapezoidal velocity: cruise `rate`, accel `max_accel`); `0` keeps the linear
 *  constant-`rate` ramp (ADR-032). Domain-agnostic: the caller interprets the output as
 *  a velocity (rad/s; rate = rad/s^2) or a position (rad; rate = rad/s). The scheduler feeds it to the
 *  loop selected by `test_mode` (0x2910), while that loop is enabled.
 */

typedef enum
{
    MC_SIG_IDLE = 0,
    MC_SIG_RAMP,      /**< ramping toward the current target (a peak, or 0) */
    MC_SIG_DWELL      /**< holding at the peak for the dwell time */
} MC_SigState_t;

typedef struct
{
    MC_SigState_t state;
    float peak;            /**< signed peak of the current pulse (sign flips each cycle if continuous) */
    float rate;            /**< ramp rate [units/s]; <= 0 => instantaneous edges (step) */
    float dwell_s;         /**< hold time at the peak [s] */
    float pause_s;         /**< inter-pulse pause at 0 between pulses [s] (continuous mode) */
    float max_accel;       /**< >0: accel-limited (trapezoidal) ramp [units/s^2]; 0: constant-rate ramp */
    bool  continuous;      /**< false: one-shot pulse; true: alternating train (ping-pong) */
    float target;          /**< current ramp target (peak or 0) */
    bool  to_peak;         /**< true: ramping out to a peak; false: ramping back to 0 */
    float value;           /**< current output */
    float vel;             /**< current rate-of-change of the output [units/s] -- the FF velocity */
    float dwell_elapsed_s;
} MC_SignalGen_t;

void  MC_SignalGen_Init  (MC_SignalGen_t *g);
/** Start a pulse: ramp to @p amplitude (signed) at @p rate, hold @p dwell_s, return to 0.
 *  @p rate <= 0 => step edges; @p continuous => alternate the peak sign and repeat, pausing @p pause_s
 *  at 0 between pulses. @p max_accel > 0 => acceleration-limited (trapezoidal) ramps; 0 => linear. */
void  MC_SignalGen_Start (MC_SignalGen_t *g, float amplitude, float rate, float dwell_s, float pause_s,
                          float max_accel, bool continuous);
/** Request a graceful stop: stop repeating and ramp the output back to 0, then idle (bumpless). */
void  MC_SignalGen_Stop  (MC_SignalGen_t *g);
bool  MC_SignalGen_Active(const MC_SignalGen_t *g);
/** Current rate-of-change of the output [units/s] -- use as the velocity feedforward when the output is
 *  a position reference. ±`rate` during a ramp; 0 during dwell, a step edge, or idle. */
float MC_SignalGen_Velocity(const MC_SignalGen_t *g);
/** Advance by @p dt_s; returns the current reference output. */
float MC_SignalGen_Update(MC_SignalGen_t *g, float dt_s);

#endif /* MC_SIGNAL_GEN_H */
