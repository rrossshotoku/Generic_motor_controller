#ifndef MC_CURRENT_SENSE_H
#define MC_CURRENT_SENSE_H
#include "mc_types.h"

/** @ingroup mc_backend */
typedef struct
{
    float offset_a_counts;
    float offset_b_counts;
    float offset_c_counts;
    float amps_per_count_a;
    float amps_per_count_b;
    float amps_per_count_c;
    bool calibrated;
} MC_CurrentSense_t;

void MC_CurrentSense_Init(MC_CurrentSense_t *cs);
bool MC_CurrentSense_CalibrateOffsets(MC_CurrentSense_t *cs, uint16_t sample_count);
bool MC_CurrentSense_ReadFast(MC_CurrentSense_t *cs, MC_PhaseCurrents_t *currents);

#endif
