# ADR-011: FOC current loop (D1)

## Status

Accepted

## Date

2026-06-19

## Context

First closed-loop torque control: regulate the d/q currents. The proven `bldc_axis_controller`
FOC is the reference (verified from code).

## Decision

- **`mc_pid.c`** (reusable PID): P/PI/PID via config flags, integrator clamp + output clamp
  anti-windup, optional derivative-on-measurement with low-pass. Used by current (and later
  velocity/position) loops.
- **`mc_foc.c`**: amplitude-invariant Clarke → Park → two PI loops (id, iq) → circular voltage
  limit → inverse Park → **SVPWM** (per-unit inverse Clarke + min/max zero-sequence injection,
  duty clamp 0.95). Current PI defaults kp 1.7, ki 1700, output ∓24 V (ported).
- **Integration** (fast loop, 20 kHz): when `inject_enable && foc_enable` (and not tripped),
  run FOC with `id_cmd`/`iq_cmd` from the watch window, currents from the current-sense, and
  the **electrical angle published by the medium-loop estimator** via a single `volatile float`
  (atomic on M4 — the rate-crossing hand-off for the angle). Over-current trip still applies.
  Open-loop C2 drive remains as the `else` branch.

## Reasoning

Matches the validated FOC math and gains. Publishing the angle as one atomic float avoids
tearing without a full double-buffer. Commanding iq from the watch window with the over-current
trip gives a controllable first-torque test.

## Consequences

- New `mc_pid.c`, `mc_foc.c`; fast loop gains an FOC mode; new watch/inject fields
  (foc_enable, iq_cmd_a, id_cmd_a; id_meas_a, iq_meas_a, vd_v, vq_v, voltage_saturated).
- **Electrical angle is the latest 1 kHz value** (up to 1 ms stale) — fine at low speed; a
  fast-loop angle predictor (extrapolate by electrical velocity) is the next refinement for
  higher speed.
- **Dead-time compensation not included** (deadband ~1 V); add later only if needed (user
  decision). An unloaded motor will accelerate to the bus-voltage speed limit under fixed iq,
  so first light should hold/load the rotor or be ready to disable.

## Files affected

- src/mc_pid.c, src/mc_foc.c, include/mc_foc.h, include/mc_pid.h
- include/mc_debug.h, src/mc_scheduler.c
- docs/spec/09_control_loops.md, docs/spec/10_foc_backend_stm32g474.md

## Open questions

- Fast-loop electrical-angle prediction (needed above low speed).
- Final current-loop gains for this motor (tune on bench).
- iq sign vs +CW torque — verify on first light; flip if needed.
