#ifndef MC_CURRENT_SENSE_H
#define MC_CURRENT_SENSE_H
#include "mc_types.h"

/** @file mc_current_sense.h
 *  @brief Three-shunt phase-current sensing interface (board-independent).
 *  @ingroup mc_backend
 *
 *  Converts raw ADC counts to phase currents in amps using per-board scaling and a
 *  zero-current offset. The hardware read is implemented in the STM32 boundary module
 *  (mc_current_sense_stm32g474.c); this interface is HAL-free. Phase B is derived from
 *  A and C by Kirchhoff's law. See ADR-007.
 */

/** @brief Current-sense calibration + scaling state for one axis. */
typedef struct
{
    float    offset_a_counts;   /**< Phase A zero-current ADC offset [counts]. */
    float    offset_b_counts;   /**< Phase B offset [counts] (unused; B is derived). */
    float    offset_c_counts;   /**< Phase C zero-current ADC offset [counts]. */
    float    amps_per_count_a;  /**< Phase A amps per count (sign-folded). */
    float    amps_per_count_b;  /**< Phase B amps per count (sign-folded). */
    float    amps_per_count_c;  /**< Phase C amps per count (sign-folded). */
    uint16_t last_raw_a;        /**< Most recent phase A raw ADC counts (observation). */
    uint16_t last_raw_c;        /**< Most recent phase C raw ADC counts (observation). */
    bool     calibrated;        /**< Offsets have been measured. */
} MC_CurrentSense_t;

/** @brief Initialise scaling from the active board profile; offsets default to nominal. */
void MC_CurrentSense_Init(MC_CurrentSense_t *cs);

/**
 * @brief Accumulate one zero-current sample toward an offset calibration.
 *
 * Call once per fast cycle with the power stage in safe-off (no current flowing). Averages
 * @p sample_count samples; sets the offsets and @c calibrated when complete.
 * @param cs           Current-sense state.
 * @param sample_count Samples to average (e.g. 2000 ~= 0.1 s at 20 kHz).
 * @return true once calibration is complete, false while still accumulating.
 */
bool MC_CurrentSense_CalibrateOffsets(MC_CurrentSense_t *cs, uint16_t sample_count);

/**
 * @brief Read the latest simultaneous phase-current sample and convert to amps.
 * @param cs       Current-sense state.
 * @param currents Output phase currents [A] (B derived) + validity + timestamp.
 * @return true if the sample is valid (no ADC overrun).
 */
bool MC_CurrentSense_ReadFast(MC_CurrentSense_t *cs, MC_PhaseCurrents_t *currents);

#endif /* MC_CURRENT_SENSE_H */
