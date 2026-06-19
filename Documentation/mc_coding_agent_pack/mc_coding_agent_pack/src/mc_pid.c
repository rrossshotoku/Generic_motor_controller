#include "mc_pid.h"
#include "mc_math.h"
void MC_Pid_Init(MC_Pid_t *pid) { if (pid) { *pid = (MC_Pid_t){0}; pid->enabled = true; } }
void MC_Pid_Reset(MC_Pid_t *pid) { if (pid) { pid->integrator = 0.0f; pid->previous_error = 0.0f; pid->previous_measurement = 0.0f; pid->derivative_state = 0.0f; pid->output = 0.0f; pid->has_previous = false; } }
void MC_Pid_SetEnabled(MC_Pid_t *pid, bool enabled) { if (pid) { pid->enabled = enabled; if (!enabled) MC_Pid_Reset(pid); } }
float MC_Pid_Update(MC_Pid_t *pid, const MC_PidConfig_t *cfg, float setpoint, float measurement) { return MC_Pid_UpdateError(pid, cfg, setpoint - measurement, measurement); }
float MC_Pid_UpdateError(MC_Pid_t *pid, const MC_PidConfig_t *cfg, float error, float measurement_for_derivative) {
    if (!pid || !cfg || !pid->enabled || cfg->sample_period_s <= 0.0f) return 0.0f;
    float p = cfg->kp * error;
    if (cfg->integrator_enabled) {
        pid->integrator += cfg->ki * error * cfg->sample_period_s;
        pid->integrator = MC_Math_Clamp(pid->integrator, cfg->integrator_min, cfg->integrator_max);
    }
    float d = 0.0f;
    if (cfg->derivative_enabled && pid->has_previous) {
        float raw_d = cfg->derivative_on_measurement ? -(measurement_for_derivative - pid->previous_measurement) / cfg->sample_period_s : (error - pid->previous_error) / cfg->sample_period_s;
        if (cfg->derivative_filter_hz > 0.0f) {
            float alpha = cfg->sample_period_s * cfg->derivative_filter_hz;
            if (alpha > 1.0f) alpha = 1.0f;
            pid->derivative_state += alpha * (raw_d - pid->derivative_state);
            raw_d = pid->derivative_state;
        }
        d = cfg->kd * raw_d;
    }
    pid->previous_error = error;
    pid->previous_measurement = measurement_for_derivative;
    pid->has_previous = true;
    pid->output = MC_Math_Clamp(p + pid->integrator + d, cfg->output_min, cfg->output_max);
    return pid->output;
}
