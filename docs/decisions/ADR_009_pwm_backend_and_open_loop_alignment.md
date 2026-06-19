# ADR-009: PWM backend + open-loop d-axis drive and rotor alignment (C1/C2)

## Status

Accepted

## Date

2026-06-19

## Context

First drive into the motor. We need a 3-phase PWM output backend and a safe way to apply a
controlled voltage vector to align the rotor and capture the electrical offset. The proven
`alignment.c` applied a d-axis voltage at electrical angle 0 (inverse Clarke: Va=Vd,
Vb=Vc=-Vd/2; duty = 0.5 + V/Vbus), ramped/held, then recorded the encoder position as
electrical 0.

## Decision

- **PWM backend** (`mc_pwm_stm32g474.c`): TIM1 time base runs continuously (ADC trigger);
  output drive is gated by the **main output enable (MOE)**. `MC_Pwm_Start` arms the channels +
  sets MOE; `MC_Pwm_ForceSafeOff` clears MOE (immediate, ISR-safe safe-off) and sets neutral
  duties; `MC_Pwm_SetDutyFast` writes centre-aligned compare values from duties in [0,1].
- **Open-loop drive** (fast loop): apply a commanded d-axis voltage `Vd` at a commanded
  electrical angle `theta` (inverse Park with Vq=0 -> inverse Clarke -> duty = 0.5 + V/Vbus).
  For first light `Vd` is **manually commanded** from the watch window (the operator ramps it),
  rather than the old blocking auto-ramp.
- **Safety**: all drive gated by `g_mc_inject.inject_enable`; `Vd` clamped to <= 3 V and duties
  to [0,1]; a **latched over-current trip** (`g_mc_inject.current_limit_a`, default 2 A) forces
  safe-off; `clear_fault` resets it. Bus voltage for the duty calc is supplied via
  `g_mc_inject.vbus_v` (we do not measure Vbus).
- **Alignment capture** (medium loop): on `request_align_capture`, set the estimator
  `electrical_offset_rad = wrap(-single * pole_pairs)` so the electrical angle reads 0 at the
  held rotor position. (Mechanical "home" / persistence is a separate Phase-E concern.)

## Reasoning

Matches the validated alignment method (inverse Clarke at theta=0). MOE gating gives an
instant, ISR-safe kill. Manual Vd ramp gives the operator direct control and an immediate stop
for first light, with the over-current trip and a current-limited bench supply as backstops.

## Consequences

- New `mc_pwm_stm32g474.c` (PWM backend) and `mc_math.c` (libm wrappers; was missing).
- The A1.5 bench-PWM (`BenchTest_ServicePwm`) is removed -- the fast loop now owns PWM, so the
  two cannot fight over TIM1.
- New `g_mc_inject` fields (align_voltage_v, align_angle_rad, vbus_v, current_limit_a,
  request_align_capture, clear_fault) and `g_mc_debug` fields (vd_applied_v, i_max_a,
  overcurrent_trip, elec_offset_rad).
- The captured electrical offset is held in RAM (not yet persisted); persistence is Phase E.

## Files affected

- include/mc_pwm.h (existing), src/mc_pwm_stm32g474.c, src/mc_math.c
- include/mc_debug.h, src/mc_scheduler.c, Core/Src/main.c

## Open questions

- Auto-ramp + settle-detect alignment routine and persistence of the offset (Phase E / D1).
- Phase-order verification (sweep theta open-loop and confirm the rotor follows +theta).
- SVPWM (vs the simple inverse-Clarke centred modulation) for the FOC voltage stage (D1).
