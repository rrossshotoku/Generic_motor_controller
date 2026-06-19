#ifndef MC_PID_H
#define MC_PID_H

#include "mc_types.h"

/** @file mc_pid.h
 *  @brief Reusable PID/PI primitive used by position, velocity and current loops.
 *  @ingroup mc_control
 */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float sample_period_s;
    float output_min;
    float output_max;
    float integrator_min;
    float integrator_max;
    float derivative_filter_hz;
    bool integrator_enabled;
    bool derivative_enabled;
    bool derivative_on_measurement;
} MC_PidConfig_t;

typedef struct
{
    float integrator;
    float previous_error;
    float previous_measurement;
    float derivative_state;
    float output;
    bool has_previous;
    bool enabled;
} MC_Pid_t;

void MC_Pid_Init(MC_Pid_t *pid);
void MC_Pid_Reset(MC_Pid_t *pid);
void MC_Pid_SetEnabled(MC_Pid_t *pid, bool enabled);
float MC_Pid_Update(MC_Pid_t *pid, const MC_PidConfig_t *cfg, float setpoint, float measurement);
float MC_Pid_UpdateError(MC_Pid_t *pid, const MC_PidConfig_t *cfg, float error, float measurement_for_derivative);

#endif
