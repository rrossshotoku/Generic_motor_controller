#ifndef MC_CALIBRATION_H
#define MC_CALIBRATION_H
#include "mc_types.h"

typedef enum {
    MC_CAL_NONE = 0,
    MC_CAL_CURRENT_OFFSETS,
    MC_CAL_ENCODER_ALIGNMENT,
    MC_CAL_PHASE_ORDER,
    MC_CAL_ELECTRICAL_OFFSET,
    MC_CAL_SOFT_LIMITS,
    MC_CAL_HOMING
} MC_CalibrationRoutine_t;

typedef enum { MC_CAL_IDLE, MC_CAL_RUNNING, MC_CAL_DONE, MC_CAL_FAILED, MC_CAL_ABORTED } MC_CalibrationState_t;
void MC_Calibration_Start(MC_CalibrationRoutine_t routine);
void MC_Calibration_Update(void);
void MC_Calibration_Abort(void);
MC_CalibrationState_t MC_Calibration_GetState(void);
#endif
