#ifndef MC_SSI_ENCODER_H
#define MC_SSI_ENCODER_H
#include "mc_position_sensor.h"

typedef struct {
    uint8_t total_bits;
    uint8_t position_bits;
    uint8_t position_lsb;
    int8_t error_bit;
    int8_t warning_bit;
    bool parity_enabled;
    bool parity_even;
    float counts_per_rev;
    float mechanical_zero_offset_rad;
    int8_t direction;
    float sample_period_s;
} MC_SsiEncoderConfig_t;

typedef struct { MC_PositionSensorSample_t last_sample; bool initialised; } MC_SsiEncoder_t;
void MC_SsiEncoder_Init(MC_SsiEncoder_t *enc, const MC_SsiEncoderConfig_t *cfg);
bool MC_SsiEncoder_Read(MC_SsiEncoder_t *enc, const MC_SsiEncoderConfig_t *cfg, MC_PositionSensorSample_t *sample);
#endif
