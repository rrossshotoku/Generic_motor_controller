# ADR-032: Acceleration-limited tuning profiles (trapezoidal velocity)

- **Status:** Accepted
- **Date:** 2026-06-24
- **Related:** ADR-030 (loop-tuning modes + signal generator), ADR-031 (velocity FF gain + signal-gen velocity)

## Context

The position-tuning signal generator (ADR-030) ramps the **position** reference at a constant
velocity `rate` (rad/s). Its velocity is therefore a rectangular pulse (`0 → rate → 0`), so the
commanded **acceleration is impulsive** at the ramp corners — an instantaneous (effectively infinite)
acceleration demand. In the cascade that shows up as a step in the velocity demand (the FF steps
`0 → rate`), which momentarily **saturates the velocity-loop current** at the corners
(`±vel_current_limit_a`).

Velocity tuning does **not** have this problem: there `rate` is in rad/s² and is already the
*acceleration* of the velocity ramp, so the corner is a finite torque step (`J·rate`), not an impulse.

## Decision

Add acceleration limiting to the **position-tuning** generator via a new motor-owned OD parameter
**`test_max_accel` (0x2910:10, F32, RW, rad/s²)**. When `max_accel > 0` (and `rate > 0`) the generator
drives `value` with a **trapezoidal velocity profile**: accelerate at `max_accel` up to cruise speed
`rate`, cruise, then decelerate at `max_accel` to stop exactly at `amplitude` (triangular if the move is
too short to reach cruise). The motor's commanded acceleration is thereby bounded by `max_accel`, and the
velocity FF (`MC_SignalGen_Velocity`, ADR-031) automatically follows the shaped, ramped velocity — so the
impulsive corner is gone.

`max_accel = 0` (or `rate = 0`, the step case) → the previous **linear-ramp** behaviour (backward
compatible).

**Scope — position tuning only.** The scheduler passes `test_max_accel` to the generator when arming
position tuning and **0** when arming velocity tuning (velocity tuning's `rate` is already its
acceleration; left unchanged). The generator itself is mode-agnostic — it trapezoids whenever
`max_accel > 0` — so the policy lives in the scheduler, not the module.

**Shape — trapezoidal** (acceleration-limited, finite jerk at the accel corners) — matches the trajectory
planner. An S-curve (jerk-limited) variant was considered and **deferred**.

Mechanics: the trapezoid uses the standard braking law — the fastest speed from which the profile can
still stop at the target is `v_allow = sqrt(2·max_accel·dist)`; each tick the speed ramps (at
`max_accel`) toward `min(rate, v_allow)`, then `value += vel·dt`. This naturally yields accelerate →
cruise → decelerate and stops at the target for both the long (trapezoid) and short (triangle) cases.

## Consequences

- Position-tuning moves become accel-limited: you set a max acceleration the system can handle and the
  demanded profile (position **and** the FF velocity) stays within it — no corner current-saturation.
- `mc_signal_gen` gains a small trapezoidal sub-profiler and now uses `<math.h>` (`sqrtf`/`fabsf`).
  It still runs in the 1 kHz medium loop; one `sqrtf` per tick is negligible on the FPU.
- Additive contract entry `0x2910:10` (non-PDO) → `MC_IF_PROTOCOL_VERSION` stays **3**. CHANGELOG
  [3.11.0]. No CMC change (motor-owned, SPI-forwarded).
- Velocity tuning behaviour unchanged.

## Files

`include/mc_signal_gen.{h,c}`, `include/mc_od_store.h`, `src/mc_scheduler.c`,
`../Lightweight_CMC/Interface/mc_if_od.h` + `CHANGELOG.md`, `gui/mc_gui/main_window.py`,
docs (ADR-030 OD block, `ADR_000_decision_log.md`, `requirements.yaml`, `docs/spec/09_control_loops.md`).
