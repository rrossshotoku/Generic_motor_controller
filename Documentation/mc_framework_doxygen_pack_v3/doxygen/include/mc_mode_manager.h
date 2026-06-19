#ifndef MC_MODE_MANAGER_H
#define MC_MODE_MANAGER_H
#include "mc_types.h"
#include "mc_faults.h"

typedef enum {
    MC_MODE_DISABLED = 0,
    MC_MODE_POSITION_HOLD,
    MC_MODE_PROFILE_POSITION,
    MC_MODE_JOYSTICK_VELOCITY,
    MC_MODE_PROFILE_VELOCITY,
    MC_MODE_TORQUE_CURRENT,
    MC_MODE_HOMING_CALIBRATION,
    MC_MODE_QUICK_STOP,
    MC_MODE_FAULT
} MC_Mode_t;

typedef struct { uint16_t controlword; int8_t requested_mode; } MC_ModeCommand_t;
typedef struct { uint16_t statusword; MC_Mode_t active_mode; } MC_ModeStatus_t;
void MC_ModeManager_Init(void);
void MC_ModeManager_Update(const MC_ModeCommand_t *cmd, const MC_FaultState_t *faults);
MC_ModeStatus_t MC_ModeManager_GetStatus(void);
#endif
