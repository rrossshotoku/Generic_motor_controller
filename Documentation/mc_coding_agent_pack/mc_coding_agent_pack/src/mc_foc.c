#include "mc_foc.h"
#include "mc_math.h"
void MC_Foc_Init(MC_Foc_t *foc, const MC_FocConfig_t *cfg) { (void)cfg; if (foc) { *foc = (MC_Foc_t){0}; MC_Pid_Init(&foc->id_loop); MC_Pid_Init(&foc->iq_loop); } }
void MC_Foc_Reset(MC_Foc_t *foc) { if (foc) { MC_Pid_Reset(&foc->id_loop); MC_Pid_Reset(&foc->iq_loop); foc->vd_v = foc->vq_v = 0.0f; foc->voltage_saturated = false; } }
MC_PwmDuty_t MC_Foc_Update(MC_Foc_t *foc, const MC_FocConfig_t *cfg, const MC_FocCurrentCommand_t *cmd, const MC_PhaseCurrents_t *currents, const MC_ElectricalState_t *electrical, float bus_voltage_v) { MC_PwmDuty_t duty = {0.5f,0.5f,0.5f,false}; if (!foc || !cfg || !cmd || !currents || !electrical || !cmd->enable || !currents->valid || !electrical->electrical_valid || bus_voltage_v <= 0.0f) { return duty; } /* TODO: implement Clarke/Park, PI, voltage limiting, inverse Park, SVPWM. */ duty.enable = true; return duty; }
