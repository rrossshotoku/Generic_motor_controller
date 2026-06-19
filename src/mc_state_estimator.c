#include "mc_state_estimator.h"
#include <math.h>

/** @file mc_state_estimator.c
 *  @brief Mechanical/electrical state estimation from a position sample. See ADR-008.
 *
 *  Builds a continuous (multi-turn) mechanical position from a single-turn absolute sample,
 *  estimates mechanical velocity by finite-difference + first-order low-pass, and derives the
 *  electrical angle for FOC. The position-tracking velocity observer (ADR-003) is added in a
 *  later increment (B2b).
 */

#define MC_EST_TWO_PI 6.28318530717958647692f
#define MC_EST_PI     3.14159265358979323846f

static float wrap_pi(float a)
{
    while (a >  MC_EST_PI) { a -= MC_EST_TWO_PI; }
    while (a < -MC_EST_PI) { a += MC_EST_TWO_PI; }
    return a;
}

static float wrap_2pi(float a)
{
    while (a >= MC_EST_TWO_PI) { a -= MC_EST_TWO_PI; }
    while (a < 0.0f)           { a += MC_EST_TWO_PI; }
    return a;
}

void MC_StateEstimator_Init(MC_StateEstimator_t *est)
{
    est->mechanical.position_rad             = 0.0f;
    est->mechanical.velocity_rad_per_s       = 0.0f;
    est->mechanical.acceleration_rad_per_s2  = 0.0f;
    est->mechanical.timestamp_ticks          = 0u;
    est->mechanical.position_valid           = false;
    est->mechanical.velocity_valid           = false;
    est->mechanical.absolute_position_valid  = false;

    est->electrical.electrical_angle_rad        = 0.0f;
    est->electrical.electrical_velocity_rad_per_s = 0.0f;
    est->electrical.electrical_valid            = false;

    est->continuous_position_rad = 0.0f;
    est->prev_single_rad         = 0.0f;
    est->velocity_filtered       = 0.0f;
    est->has_prev                = false;
}

void MC_StateEstimator_Update(MC_StateEstimator_t *est,
                              const MC_StateEstimatorConfig_t *cfg,
                              const MC_PositionSensorSample_t *sample)
{
    const float single = sample->position_rad;   /* corrected single-turn [0, 2pi) */
    const float dt = cfg->sample_period_s;

    if (!est->has_prev)
    {
        est->continuous_position_rad = single;
        est->prev_single_rad         = single;
        est->velocity_filtered       = 0.0f;
        est->has_prev                = true;
    }
    else
    {
        float delta = wrap_pi(single - est->prev_single_rad);   /* handles single-turn wrap */
        est->prev_single_rad          = single;
        est->continuous_position_rad += delta;

        const float raw_vel = (dt > 0.0f) ? (delta / dt) : 0.0f;
        const float fc = cfg->velocity_filter_hz;
        const float alpha = (fc > 0.0f && dt > 0.0f)
                          ? (1.0f - expf(-MC_EST_TWO_PI * fc * dt))
                          : 1.0f;
        est->velocity_filtered += alpha * (raw_vel - est->velocity_filtered);
    }

    est->mechanical.position_rad            = est->continuous_position_rad;
    est->mechanical.velocity_rad_per_s      = est->velocity_filtered;
    est->mechanical.acceleration_rad_per_s2 = 0.0f;   /* B2: not estimated */
    est->mechanical.timestamp_ticks         = sample->timestamp_ticks;
    est->mechanical.position_valid          = sample->valid;
    est->mechanical.velocity_valid          = est->has_prev && sample->valid;
    est->mechanical.absolute_position_valid = sample->absolute && sample->valid;

    const float elec = wrap_2pi(single * cfg->pole_pairs + cfg->electrical_offset_rad);
    est->electrical.electrical_angle_rad          = elec;
    est->electrical.electrical_velocity_rad_per_s = est->velocity_filtered * cfg->pole_pairs;
    est->electrical.electrical_valid              = sample->valid;
}
