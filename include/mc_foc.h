#ifndef MC_FOC_H
#define MC_FOC_H
#include "mc_types.h"
#include "mc_pid.h"

/** @file mc_foc.h
 *  @brief Basic BLDC/PMSM FOC current controller.
 *  @ingroup mc_backend
 */

typedef struct
{
    MC_PidConfig_t id_pi;
    MC_PidConfig_t iq_pi;
    float voltage_limit_v;
    float sample_period_s;
    bool use_cordic_if_available;
} MC_FocConfig_t;

typedef struct
{
    MC_Pid_t id_loop;
    MC_Pid_t iq_loop;
    float id_measured_a;
    float iq_measured_a;
    float vd_v;
    float vq_v;
    bool voltage_saturated;
    bool enabled;
} MC_Foc_t;

void MC_Foc_Init(MC_Foc_t *foc, const MC_FocConfig_t *cfg);
void MC_Foc_Reset(MC_Foc_t *foc);
MC_PwmDuty_t MC_Foc_Update(MC_Foc_t *foc,
                           const MC_FocConfig_t *cfg,
                           const MC_FocCurrentCommand_t *cmd,
                           const MC_PhaseCurrents_t *currents,
                           const MC_ElectricalState_t *electrical,
                           float bus_voltage_v);

#endif
