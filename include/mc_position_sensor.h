#ifndef MC_POSITION_SENSOR_H
#define MC_POSITION_SENSOR_H
#include "mc_types.h"

/** @file mc_position_sensor.h
 *  @brief Common position sensor sample interface for SSI and future quadrature backends.
 *  @ingroup mc_feedback
 */

typedef struct
{
    float position_rad;
    uint32_t raw_position;
    uint32_t timestamp_ticks;
    bool valid;
    bool absolute;
    bool error;
    bool warning;
} MC_PositionSensorSample_t;

#endif
