#ifndef MC_PWM_H
#define MC_PWM_H
#include "mc_types.h"

/** @ingroup mc_backend */
void MC_Pwm_Start(void);
void MC_Pwm_Stop(void);
void MC_Pwm_SetDutyFast(const MC_PwmDuty_t *duty);
void MC_Pwm_ForceSafeOff(void);

#endif
