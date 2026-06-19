#ifndef MC_POSITION_SENSOR_H
#define MC_POSITION_SENSOR_H
#include "mc_types.h"

typedef struct {
    uint32_t raw_position;
    float position_rad;
    bool valid;
    bool error;
    bool warning;
    uint32_t timestamp_ticks;
} MC_PositionSensorSample_t;
#endif
