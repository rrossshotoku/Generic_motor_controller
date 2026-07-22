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

/** @brief Full persistable parameter set (ADR-023) = the calibration subset + every persistent
 *  (MC_IF_F_PERSIST) OD entry. The OD entries are serialized by MC_Od_GatherPersistent as
 *  {index(LE16), subindex, len, value} records; @ref od_blob_len bytes are valid. This is the
 *  flash store payload (was bare MC_CalibData_t in v1; MC_PARAM_STORE_VERSION bumped to 2). */
/* Capacity for the serialized PERSIST OD records ({index,sub,len,value}; 8 B per F32). MUST stay >=
   the total of every motor-owned PERSIST entry -- MC_Od_GatherPersistent SILENTLY drops entries
   once full, losing the highest-index ones. History: 256 B dropped 0x2600:6/7, 0x2700:3/4, 0x6081-5
   (ADR-044 -> 448 B); 448 B then dropped 0x2700:11 position_recall_enable + the 0x2930 notch entries
   once thermal/dither/recall/current-demand PERSIST entries pushed the total to 473 B (ADR-070 ->
   640 B, ~167 B / ~20 F32 headroom). Keep sizeof(MC_Params_t) <= MC_PARAM_STORE_MAX_PAYLOAD.
   Growing this is now BACKWARD-COMPATIBLE (no MC_PARAM_STORE_VERSION bump): od_blob is the LAST
   field, so a shorter old record front-loads and MC_PersistentStore_Read zero-fills the grown tail
   (ADR-070) -- calib + all previously-saved entries survive the update. */
#define MC_PARAMS_OD_BLOB_MAX (640u)
typedef struct
{
    MC_CalibData_t calib;                          /**< Calibration (offsets, home, phase order). */
    uint16_t       od_blob_len;                    /**< Valid bytes in od_blob. */
    uint8_t        od_blob[MC_PARAMS_OD_BLOB_MAX];  /**< Serialized persistent OD entries (MUST stay last). */
} MC_Params_t;

#endif /* MC_CALIB_DATA_H */
