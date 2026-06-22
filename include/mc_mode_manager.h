#ifndef MC_MODE_MANAGER_H
#define MC_MODE_MANAGER_H
#include "mc_types.h"
#include "mc_faults.h"

/** @ingroup mc_motion */
typedef enum
{
    MC_MODE_DISABLED = 0,
    MC_MODE_POSITION_HOLD,
    MC_MODE_PROFILE_POSITION,
    MC_MODE_PROFILE_VELOCITY,
    MC_MODE_TORQUE_CURRENT,
    MC_MODE_HOMING_CALIBRATION,
    MC_MODE_QUICK_STOP,
    MC_MODE_FAULT
} MC_Mode_t;

typedef struct
{
    uint16_t controlword;
    int8_t mode_of_operation;
    float target_position_rad;
    float target_velocity_rad_per_s;
    float target_torque_nm;
    float requested_time_s;
    bool new_setpoint;
    bool halt;
    bool fault_reset;
} MC_DriveCommand_t;

typedef struct
{
    MC_Mode_t active_mode;
    uint16_t statusword;
    bool operation_enabled;
    bool motion_active;
    bool target_reached;
    bool new_setpoint_latched;   /* one-shot: NEW_SETPOINT rising edge this update (D3 trajectory consumes) */
} MC_DriveStatus_t;

void MC_ModeManager_Init(void);
void MC_ModeManager_Update(const MC_DriveCommand_t *cmd, const MC_FaultState_t *faults);
MC_DriveStatus_t MC_ModeManager_GetStatus(void);
MC_Mode_t MC_ModeManager_GetActiveMode(void);

#endif
