#ifndef MC_THERMAL_H
#define MC_THERMAL_H
#include "mc_types.h"

/** @file mc_thermal.h
 *  @brief First-order (I²t) winding thermal model with progressive derate (ADR-065).
 *  @ingroup mc_control
 *
 *  Utilisation `x`: 0 = cold, 1 = at the rated thermal limit. The whole model is
 *      dx/dt = (1/tau) * ((I/I_cont)^2 - x)
 *  parameterised by exactly two numbers: `I_cont` [A] (steady-state limit) and
 *  `tau` [s] (thermal time constant). Model-only (current-based) — no temperature
 *  sensor. HAL-free / host-compilable. Runs in the slow loop (100 Hz).
 */

/** @brief Reset to the safe disabled state (x=0, derate=1, no fault). */
void  MC_Thermal_Init(void);

/** @brief Apply OD config (0x2100:1/2/3/6). @p derate_start is the utilisation x where the
 *  current-limit derate begins (clamped to [0, 0.99]; derate is full at x=1). Cheap; safe
 *  to call every slow tick. */
void  MC_Thermal_SetParams(bool enable, float i_cont_a, float tau_s, float derate_start);

/** @brief Advance the model by @p dt_s using the present motor current magnitude [A]. */
void  MC_Thermal_Update(float motor_current_a, float dt_s);

float MC_Thermal_Utilisation(void);    /**< x (0 = cold, 1 = at limit); 0x2100:4. */
float MC_Thermal_DerateFactor(void);   /**< current-limit multiplier 0..1 (1 when disabled); 0x2100:5. */
bool  MC_Thermal_OverTemp(void);       /**< backstop fault: x above the trip (hysteretic); MC_IF_FAULT_OVERTEMP. */

#endif /* MC_THERMAL_H */
