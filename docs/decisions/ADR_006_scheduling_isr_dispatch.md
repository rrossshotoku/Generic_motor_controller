# ADR-006: Real-time scheduling — ISR dispatch and the 20 kHz fast-loop trigger

## Status

Accepted

## Date

2026-06-19

## Context

Three timing domains must be driven (fast 20 kHz, medium 1 kHz, slow 100 Hz). CubeMX
configures TIM1 as centre-aligned PWM (PSC 0, ARR 4250, **RCR 0**) and TIM7 as a 1 kHz base
(PSC 169, ARR 999). Both peripheral ISRs route through `HAL_TIM_IRQHandler`, which invokes
the weak `HAL_TIM_PeriodElapsedCallback`. Key subtlety: with centre-aligned + RCR 0, the TIM1
**update event fires at 40 kHz** (twice per 20 kHz PWM period), not 20 kHz.

## Decision

- Dispatch via `HAL_TIM_PeriodElapsedCallback` implemented in `main.c` USER CODE: TIM1 →
  `MC_Sched_FastTick`, TIM7 → `MC_Sched_MediumTick`. No edits to generated ISR code.
- **Fast 20 kHz**: `MC_Sched_FastTick` software-divides the 40 kHz TIM1 update by 2.
  This is interim — the definitive FOC trigger moves to **ADC end-of-conversion** (once per
  PWM period, synchronized to current sampling) when current sensing lands in Stage B1.
- **Medium 1 kHz**: `MC_Sched_MediumTick` runs directly from TIM7.
- **Slow 100 Hz**: decimated from medium (÷10) and serviced in the **main loop** via
  `MC_Sched_ServiceBackground`, keeping longer work out of ISR context.
- `mc_scheduler` is **HAL-free**; the timing wrappers (`*_Tick`, `ServiceBackground`) own
  timing/decimation and call the loop bodies. `main.c` (the HAL boundary) starts the timers.

## Reasoning

The callback route needs only `main.c` USER CODE (no generated-code edits). Dividing by 2
yields a true 20 kHz call rate now; ADC-EOC is the correct, jitter-free FOC trigger once the
ADC runs. Servicing the slow loop in the background avoids long operations (OD/flash) in
interrupt context. Keeping `mc_scheduler` HAL-free preserves boundary isolation.

## Consequences

- `include/mc_scheduler.h` gains `MC_Sched_FastTick/MediumTick/ServiceBackground`.
- `main.c` USER CODE: implements the callback, starts `HAL_TIM_Base_Start_IT` for TIM1 and
  TIM7. PWM outputs stay disabled (MOE/AutomaticOutput off) — safe-off until Stage C1.
- Loop cadence and worst-case durations are mirrored in `g_mc_debug` (DWT cycle counter).
- **Do not assume TIM1 update = 20 kHz** (RCR 0 ⇒ 40 kHz). Revisit RCR=1 vs ADC-EOC in B1.

## Files affected

- include/mc_scheduler.h
- src/mc_scheduler.c
- include/mc_debug.h, src/mc_debug.c
- Core/Src/main.c (USER CODE sections)
- docs/spec/04_realtime_scheduling.md

## Open questions

- Final fast-loop trigger: ADC-EOC (preferred) vs setting TIM1 RCR=1 in CubeMX — decide in B1.
- Whether to raise the medium loop to 2 kHz later (spec allows 1–2 kHz).
