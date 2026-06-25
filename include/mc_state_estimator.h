#ifndef MC_STATE_ESTIMATOR_H
#define MC_STATE_ESTIMATOR_H
#include "mc_position_sensor.h"

/** @ingroup mc_feedback */
typedef struct
{
    float pole_pairs;
    float electrical_offset_rad;
    float velocity_filter_hz;
    float sample_period_s;
    float obs_kp;            /**< Observer proportional gain (position error -> accel). */
    float obs_ki;            /**< Observer integral gain. */
    float obs_kv;            /**< Observer velocity-damping gain. */
    float obs_filter_alpha;  /**< Observer output-velocity LPF (0..1; 1 = no filter). */
    bool  use_observer;      /**< true = observer velocity (default, ADR-003); false = finite-diff. */
} MC_StateEstimatorConfig_t;

typedef struct
{
    MC_MechanicalState_t mechanical;
    MC_ElectricalState_t electrical;
    float continuous_position_rad;  /**< Multi-turn accumulator [rad]. */
    float prev_single_rad;          /**< Previous single-turn sample [rad] (wrap detect). */
    float velocity_filtered;        /**< Finite-difference velocity (LPF) [rad/s]. */
    float obs_theta;                /**< Observer position estimate [rad]. */
    float obs_omega;                /**< Observer velocity estimate [rad/s]. */
    float obs_integral;             /**< Observer integral accumulator. */
    float velocity_observer;        /**< Observer velocity output (filtered) [rad/s]. */
    bool has_prev;
} MC_StateEstimator_t;

void MC_StateEstimator_Init(MC_StateEstimator_t *est);
void MC_StateEstimator_Update(MC_StateEstimator_t *est,
                              const MC_StateEstimatorConfig_t *cfg,
                              const MC_PositionSensorSample_t *sample);

/** Re-anchor the multi-turn accumulator to @p continuous_rad (e.g. the startup nearest-turn seed,
 *  ADR-037). Keeps @c prev_single_rad so the next delta stays small; the observer follows so there is
 *  no phantom position error. */
void MC_StateEstimator_SeedContinuous(MC_StateEstimator_t *est, float continuous_rad);

#endif
