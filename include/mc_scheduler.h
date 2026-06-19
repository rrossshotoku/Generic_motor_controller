#ifndef MC_SCHEDULER_H
#define MC_SCHEDULER_H

/** @file mc_scheduler.h
 *  @brief Real-time loop hooks and ISR/background dispatch for the three timing domains.
 *  @ingroup mc_scheduler
 *
 *  Timing domains on this board (see ADR-006):
 *   - Fast   20 kHz : ADC end-of-conversion ISR (ADC hardware-triggered by TIM1 TRGO=
 *                     OC4REF at the PWM peak) -> @ref MC_Sched_FastTick. Sample-synchronised.
 *   - Medium  1 kHz : TIM7 update event -> @ref MC_Sched_MediumTick.
 *   - Slow  100 Hz : decimated from medium (/10), serviced in the main loop via
 *                    @ref MC_Sched_ServiceBackground (kept out of ISR context).
 *
 *  The `*_Tick` / `ServiceBackground` wrappers own timing and decimation and call the loop
 *  bodies below. They are invoked from `main.c` (the HAL boundary); this header is HAL-free.
 */

/** @brief One-time framework/software init (debug harness, modules). HAL-free. */
void MC_Framework_Init(void);

/** @brief Fast control work (20 kHz) — current loop / FOC path. */
void MC_FastLoop_20kHz(void);
/** @brief Medium motion work (1 kHz) — sensing, trajectory, position/velocity loops. */
void MC_MotionLoop_1kHz(void);
/** @brief Slow supervisory work (100 Hz) — OD/SPI service, faults, persistence. */
void MC_SlowLoop_10_100Hz(void);

/** @brief Call from the ADC end-of-conversion ISR (20 kHz, sample-synchronised): timing +
 *         @ref MC_FastLoop_20kHz. */
void MC_Sched_FastTick(void);
/** @brief Call from the TIM7 update ISR: timing + @ref MC_MotionLoop_1kHz + slow decimation. */
void MC_Sched_MediumTick(void);
/** @brief Call from the main loop: services the pending slow loop out of ISR context. */
void MC_Sched_ServiceBackground(void);

#endif /* MC_SCHEDULER_H */
