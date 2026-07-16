/*
 * mc_thermal — first-order (I²t) winding thermal model + progressive derate.
 * See mc_thermal.h and ADR-065. HAL-free; statics initialise to the safe
 * disabled state so MC_Thermal_DerateFactor() reads 1.0 even before Init.
 */

#include "mc_thermal.h"

/* Fault + derate-end thresholds (ADR-065). Firmware constants. The derate-START point is
   OD-configurable (0x2100:6); MC_THERMAL_DERATE_START_DEFAULT is only the power-on default. */
#define MC_THERMAL_DERATE_START_DEFAULT (0.85f)  /* default x where the derate begins (0x2100:6) */
#define MC_THERMAL_DERATE_END    (1.00f)  /* x where output is fully derated (limit -> 0)    */
#define MC_THERMAL_FAULT_SET_X   (1.05f)  /* x at which OVERTEMP latches (derate couldn't hold) */
#define MC_THERMAL_FAULT_CLR_X   (1.00f)  /* x at which OVERTEMP clears (hysteresis)         */

static bool  s_enable       = false;
static float s_i_cont_a     = 0.0f;
static float s_tau_s        = 0.0f;
static float s_derate_start = MC_THERMAL_DERATE_START_DEFAULT;  /* x where derate begins (0x2100:6) */
static float s_x            = 0.0f;   /* thermal utilisation */
static float s_derate       = 1.0f;   /* current-limit multiplier */
static bool  s_overtemp     = false;

void MC_Thermal_Init(void)
{
    s_enable       = false;
    s_i_cont_a     = 0.0f;
    s_tau_s        = 0.0f;
    s_derate_start = MC_THERMAL_DERATE_START_DEFAULT;
    s_x            = 0.0f;
    s_derate       = 1.0f;
    s_overtemp     = false;
}

void MC_Thermal_SetParams(bool enable, float i_cont_a, float tau_s, float derate_start)
{
    s_enable   = enable;
    s_i_cont_a = i_cont_a;
    s_tau_s    = tau_s;
    /* Clamp the derate-start utilisation to [0, 0.99] so the derate band (start .. 1.0) always
       has positive width -- guards the divide in MC_Thermal_Update against start == 1.0. */
    if      (derate_start < 0.0f)  { derate_start = 0.0f;  }
    else if (derate_start > 0.99f) { derate_start = 0.99f; }
    s_derate_start = derate_start;
}

void MC_Thermal_Update(float motor_current_a, float dt_s)
{
    if (!s_enable || (s_i_cont_a <= 0.0f) || (dt_s <= 0.0f))
    {
        /* Disabled or unconfigured -> no protection, full output, cold state. */
        s_x        = 0.0f;
        s_derate   = 1.0f;
        s_overtemp = false;
        return;
    }

    /* Steady-state target utilisation for this current: (I / I_cont)^2. */
    const float ratio  = motor_current_a / s_i_cont_a;
    const float target = ratio * ratio;

    /* First-order lag toward target. tau <= 0 => instantaneous (no burst tolerance):
       x tracks the steady-state target directly -> a plain limit at I_cont. */
    if (s_tau_s > 0.0f)
    {
        float a = dt_s / s_tau_s;
        if (a > 1.0f) { a = 1.0f; }          /* guard dt > tau: stays stable and monotone */
        s_x += a * (target - s_x);
    }
    else
    {
        s_x = target;
    }
    if (s_x < 0.0f) { s_x = 0.0f; }

    /* Progressive derate: 1 below the (configurable) start, linear to 0 at DERATE_END. */
    if (s_x <= s_derate_start)
    {
        s_derate = 1.0f;
    }
    else if (s_x >= MC_THERMAL_DERATE_END)
    {
        s_derate = 0.0f;
    }
    else
    {
        s_derate = (MC_THERMAL_DERATE_END - s_x)
                 / (MC_THERMAL_DERATE_END - s_derate_start);
    }

    /* OVERTEMP backstop with hysteresis: only if the derate could not hold x down. */
    if      (s_x >= MC_THERMAL_FAULT_SET_X) { s_overtemp = true;  }
    else if (s_x <  MC_THERMAL_FAULT_CLR_X) { s_overtemp = false; }
}

float MC_Thermal_Utilisation(void)  { return s_x; }
float MC_Thermal_DerateFactor(void) { return (s_enable && (s_i_cont_a > 0.0f)) ? s_derate : 1.0f; }
bool  MC_Thermal_OverTemp(void)     { return s_overtemp; }
