#ifndef MC_STATE_ESTIMATOR_H
#define MC_STATE_ESTIMATOR_H
#include "mc_position_sensor.h"

typedef struct {
    float pole_pairs;
    float electrical_offset_rad;
    float velocity_filter_hz;
    float sample_period_s;
} MC_StateEstimatorConfig_t;

typedef struct {
    MC_MechanicalState_t mechanical;
    MC_ElectricalState_t electrical;
    float prev_position_rad;
    bool has_prev;
} MC_StateEstimator_t;

void MC_StateEstimator_Init(MC_StateEstimator_t *est);
void MC_StateEstimator_Update(MC_StateEstimator_t *est, const MC_StateEstimatorConfig_t *cfg, const MC_PositionSensorSample_t *sample);
#endif
