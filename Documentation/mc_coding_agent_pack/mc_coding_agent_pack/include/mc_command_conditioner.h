#ifndef MC_COMMAND_CONDITIONER_H
#define MC_COMMAND_CONDITIONER_H
#include "mc_types.h"

/** @ingroup mc_motion */
typedef struct
{
    float deadband;
    float scale_rad_per_s;
    float max_accel_rad_per_s2;
    float soft_limit_min_rad;
    float soft_limit_max_rad;
    float soft_limit_slowdown_distance_rad;
} MC_JoystickConditionerConfig_t;

typedef struct
{
    float last_velocity_rad_per_s;
} MC_VelocityConditioner_t;

float MC_JoystickConditioner_Update(MC_VelocityConditioner_t *state,
                                    const MC_JoystickConditionerConfig_t *cfg,
                                    float joystick_normalised,
                                    float actual_position_rad,
                                    float dt_s);

float MC_ProfileVelocityConditioner_Update(MC_VelocityConditioner_t *state,
                                           float target_velocity_rad_per_s,
                                           float max_accel_rad_per_s2,
                                           float actual_position_rad,
                                           float soft_min_rad,
                                           float soft_max_rad,
                                           float slowdown_distance_rad,
                                           float dt_s);

#endif
