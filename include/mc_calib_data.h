#ifndef MC_CALIB_DATA_H
#define MC_CALIB_DATA_H
#include "mc_types.h"

/** @file mc_calib_data.h
 *  @brief Persistable calibration results (the flash payload for the calibration-only store).
 *  @ingroup mc_persistence
 *
 *  This is the calibration subset of the eventual full MC_Params_t (ADR-010) and is embedded
 *  there so the two stay consistent. Fixed-size, pointer-free, and a multiple of 8 bytes for
 *  double-word flash programming. Bump MC_PARAM_STORE_VERSION on any layout change.
 */
typedef struct
{
    float    electrical_offset_rad;       /**< FOC electrical-angle offset (from alignment). */
    float    current_offset_a_counts;     /**< Phase A zero-current ADC offset [counts]. */
    float    current_offset_c_counts;     /**< Phase C zero-current ADC offset [counts]. */
    float    mechanical_zero_offset_rad;  /**< Mechanical/home zero offset [rad]. */
    int32_t  phase_order;                 /**< +1 / -1 phase-order result (0 = unknown). */
    float    home_offset_rad;             /**< Multi-turn mechanical home offset [rad] (ADR-022; 0 = none).
                                               Was `reserved` (uint32 0) -- old flash reads as 0.0f, i.e.
                                               no home, so binary-compatible (no store-version bump). */
} MC_CalibData_t;   /* 24 bytes (multiple of 8) */

#endif /* MC_CALIB_DATA_H */
