# ADR-007: Current sensing — dual-ADC read, scaling, sign convention, offset calibration

## Status

Accepted

## Date

2026-06-19

## Context

Three-shunt FOC needs phase currents in amps, sampled at the PWM peak. The proven scheme in
`bldc_axis_controller` was confirmed from the code (not comments): ADC1+ADC2 dual regular-
simultaneous, TIM1-TRGO triggered, read from the common data register, with a sign convention
and offset calibration.

## Decision

- **Read**: both phases from the dual common data register via `HAL_ADCEx_MultiModeGetValue`
  (ADC1 master = phase A, ADC2 slave = phase C). Start **ADC2 (slave) before ADC1 (master,
  IT)**; the master trigger converts both.
- **Convert**: `i = (raw − offset) · amps_per_count`, where
  `amps_per_count = sign · Vref / (full_scale · gain · shunt) ≈ 0.01555 A/count` and the board
  **sign = −1** (positive current = into the phase), matching the old convention.
  **Phase B = −(A + C)** (Kirchhoff).
- **Offset calibration**: average N samples (default 2000 ≈ 0.1 s) with the power stage in
  safe-off, triggered from the watch window (`g_mc_inject.request_offset_cal`). Nominal offset
  ≈ 2122 counts until calibrated.
- **Scaling source**: board profile 0 (`MC_BoardConfig_LoadProfile0`). The HAL read lives in
  `mc_current_sense_stm32g474.c`; the interface (`mc_current_sense.h`) is HAL-free.

## Reasoning

Matches the validated reference current path and the correct sample instant (low-side
conducting at the PWM peak). Keeps boundary isolation (HAL only in the `*_stm32g474.c`).

## Consequences

- New `src/mc_current_sense_stm32g474.c` + `src/mc_board_config.c` (profile 0); `mc_current_sense.h`
  gains `last_raw_a/c` for observation.
- `main.c` now calibrates and starts ADC2 then ADC1 (the A1.5 single-ADC start is corrected).
- The fast loop reads currents each cycle and mirrors `ia/ib/ic`, raw, offsets, and validity to
  `g_mc_debug`; offset calibration runs via the inject flag.
- The **sign convention is fixed here** and the FOC (Stage D1) relies on it.

## Files affected

- include/mc_current_sense.h
- src/mc_current_sense_stm32g474.c
- src/mc_board_config.c
- include/mc_debug.h
- src/mc_scheduler.c
- Core/Src/main.c
- docs/spec/17_board_and_motor_config.md

## Open questions

- Current low-pass (IIR) filtering deferred to the FOC stage (the old firmware used alpha 0.1).
- ADC-overrun recovery: B1 flags invalid and clears the flag; a full stop/restart (as in the old
  firmware) can be added with the fault manager.
