#include "mc_pid.h"
#include <math.h>

/** @file mc_pid.c
 *  @brief Reusable PID/PI primitive (P/PI/PID via config flags, derivative filtering,
 *         integrator clamp + output clamp anti-windup). Used by the current/velocity/position
 *         loops. See ADR-011.
 */

#define MC_PID_TWO_PI 6.28318530717958647692f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

void MC_Pid_Init(MC_Pid_t *pid)
{
    pid->integrator           = 0.0f;
    pid->previous_error       = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->derivative_state     = 0.0f;
    pid->output               = 0.0f;
    pid->has_previous         = false;
    pid->enabled              = true;
}

void MC_Pid_Reset(MC_Pid_t *pid)
{
    pid->integrator           = 0.0f;
    pid->previous_error       = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->derivative_state     = 0.0f;
    pid->output               = 0.0f;
    pid->has_previous         = false;
}

void MC_Pid_SetEnabled(MC_Pid_t *pid, bool enabled)
{
    pid->enabled = enabled;
}

float MC_Pid_UpdateError(MC_Pid_t *pid, const MC_PidConfig_t *cfg,
                         float error, float measurement_for_derivative)
{
    if (!pid->enabled)
    {
        pid->output = 0.0f;
        return 0.0f;
    }

    const float dt = cfg->sample_period_s;
    const float p = cfg->kp * error;

    if (cfg->integrator_enabled)
    {
        pid->integrator += cfg->ki * error * dt;
        pid->integrator = clampf(pid->integrator, cfg->integrator_min, cfg->integrator_max);
    }
    else
    {
        pid->integrator = 0.0f;
    }

    float d = 0.0f;
    if (cfg->derivative_enabled)
    {
        float deriv = 0.0f;
        if (pid->has_previous && (dt > 0.0f))
        {
            deriv = cfg->derivative_on_measurement
                  ? (-(measurement_for_derivative - pid->previous_measurement) / dt)
                  : ((error - pid->previous_error) / dt);
        }
        const float fc = cfg->derivative_filter_hz;
        if ((fc > 0.0f) && (dt > 0.0f))
        {
            const float a = 1.0f - expf(-MC_PID_TWO_PI * fc * dt);
            pid->derivative_state += a * (deriv - pid->derivative_state);
        }
        else
        {
            pid->derivative_state = deriv;
        }
        d = cfg->kd * pid->derivative_state;
    }

    pid->previous_error       = error;
    pid->previous_measurement = measurement_for_derivative;
    pid->has_previous         = true;

    pid->output = clampf(p + pid->integrator + d, cfg->output_min, cfg->output_max);
    return pid->output;
}

float MC_Pid_Update(MC_Pid_t *pid, const MC_PidConfig_t *cfg, float setpoint, float measurement)
{
    return MC_Pid_UpdateError(pid, cfg, setpoint - measurement, measurement);
}
