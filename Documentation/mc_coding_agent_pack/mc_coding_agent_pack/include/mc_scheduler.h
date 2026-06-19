#ifndef MC_SCHEDULER_H
#define MC_SCHEDULER_H

/** @ingroup mc_scheduler */
void MC_Framework_Init(void);
void MC_FastLoop_20kHz(void);
void MC_MotionLoop_1kHz(void);
void MC_SlowLoop_10_100Hz(void);

#endif
