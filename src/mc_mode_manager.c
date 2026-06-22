#include "mc_mode_manager.h"
#include "mc_if_od.h"   /* controlword/statusword/mode constants (shared contract) */
#include <string.h>

/** @file mc_mode_manager.c
 *  @brief CiA-402 (simplified) drive state machine + operating-mode routing. See ADR-018.
 *
 *  Driven by the contract's simplified controlword bits (CW_ENABLE / CW_QUICK_STOP /
 *  CW_FAULT_RESET / CW_HALT), not the full multi-bit CiA-402 encoding. Produces the statusword
 *  and the active operating mode; the scheduler routes the active mode to the control loops. A
 *  severe fault latches Fault until a fault-reset edge with no fault present.
 */

static MC_DriveStatus_t s_status;
static bool             s_fault_latched;
static bool             s_prev_new_setpoint;   /* for NEW_SETPOINT rising-edge detection (v3) */

void MC_ModeManager_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.active_mode = MC_MODE_DISABLED;
    s_fault_latched      = false;
    s_prev_new_setpoint  = false;
}

void MC_ModeManager_Update(const MC_DriveCommand_t *cmd, const MC_FaultState_t *faults)
{
    const bool severe = (faults != 0) && faults->severe_active;
    if (severe)                              { s_fault_latched = true; }
    if (cmd->fault_reset && !severe)         { s_fault_latched = false; }

    MC_Mode_t mode    = MC_MODE_DISABLED;
    bool      enabled = false;
    uint16_t  sw      = 0u;

    if (s_fault_latched)
    {
        mode = MC_MODE_FAULT;
        sw   = MC_IF_SW_FAULT;
    }
    else
    {
        sw = MC_IF_SW_READY;
        const bool quick_stop = (cmd->controlword & MC_IF_CW_QUICK_STOP) == 0u;  /* 0 = QS active */
        const bool halt       = ((cmd->controlword & MC_IF_CW_HALT) != 0u) || cmd->halt;
        const bool enable_req = (cmd->controlword & MC_IF_CW_ENABLE) != 0u;

        if (quick_stop)
        {
            mode = MC_MODE_QUICK_STOP;
        }
        else if (enable_req && !halt)
        {
            enabled = true;
            sw |= MC_IF_SW_ENABLED;
            switch (cmd->mode_of_operation)
            {
                case MC_IF_MODE_TORQUE:            mode = MC_MODE_TORQUE_CURRENT;    break;
                case MC_IF_MODE_PROFILE_VELOCITY:  mode = MC_MODE_PROFILE_VELOCITY;  break;
                case MC_IF_MODE_PROFILE_POSITION:  mode = MC_MODE_PROFILE_POSITION;  break;
                default:  /* unknown / unroutable -> stay disabled */
                    mode = MC_MODE_DISABLED;
                    enabled = false;
                    sw &= (uint16_t)~MC_IF_SW_ENABLED;
                    break;
            }
        }
        else
        {
            mode = MC_MODE_DISABLED;
        }
    }

    /* NEW_SETPOINT rising edge -> one-shot trigger for the trajectory engine (D3 consumes it to
       latch the SDO-written setup and start a PROFILE_POSITION move). Velocity/torque modes are
       continuous and ignore it. Detected every update regardless of mode so the edge is never missed. */
    s_status.new_setpoint_latched = (cmd->new_setpoint && !s_prev_new_setpoint);
    s_prev_new_setpoint           = cmd->new_setpoint;

    s_status.active_mode       = mode;
    s_status.operation_enabled = enabled;
    s_status.statusword        = sw;
    s_status.motion_active     = enabled;
    s_status.target_reached    = false;   /* set by the position/trajectory layer (D3) */
}

MC_DriveStatus_t MC_ModeManager_GetStatus(void)   { return s_status; }
MC_Mode_t        MC_ModeManager_GetActiveMode(void){ return s_status.active_mode; }
