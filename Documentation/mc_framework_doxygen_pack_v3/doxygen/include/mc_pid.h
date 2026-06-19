#ifndef MC_PID_H
#define MC_PID_H
#include "mc_types.h"

typedef struct {
    float kp, ki, kd;
    float sample_period_s;
    float output_min, output_max;
    float integrator_min, integrator_max;
    float derivative_filter_hz;
    float slew_rate_limit_per_s;
    bool integrator_enabled;
    bool derivative_enabled;
    bool derivative_on_measurement;
} MC_PidConfig_t;

typedef struct {
    float integrator;
    float prev_error;
    float prev_measurement;
    float derivative_state;
    float prev_output;
    bool enabled;
} MC_Pid_t;

void MC_Pid_Init(MC_Pid_t *pid);
void MC_Pid_Reset(MC_Pid_t *pid);
float MC_Pid_Update(MC_Pid_t *pid, const MC_PidConfig_t *cfg,
                    float setpoint, float measurement, bool allow_integrator);
#endif
