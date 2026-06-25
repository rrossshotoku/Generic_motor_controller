# ADR-031: Velocity feedforward gain + signal-generator velocity (position-tuning FF)

## Status

Accepted

## Date

2026-06-24

## Context

The position cascade (ADR-028) feeds the trajectory's planned velocity forward into the velocity loop:
`s_eff_vel_cmd = v_ff + vcorr` (`v_ff = sp.velocity_rad_per_s`). Two gaps:

1. The feedforward is a hardcoded **1:1** sum — there is no gain to trim the FF ratio, which is the
   standard knob for position-loop tuning (set FF ≈ 1.0 to flatten following error on constant-velocity
   segments, back off if it overshoots).
2. **Position-tuning** (ADR-030) drives the position controller from the signal generator but with
   `v_ff = 0` — so a tuning ramp has no velocity feedforward, unlike a real trajectory move. The user
   wants the position-tuning path to feed forward velocity too, so it behaves like a real move.

## Decision

**1. `velocity_ff_gain` (0x2200:4, F32, RW, PERSIST, default 1.0).** The position cascade applies it to
the velocity feedforward: `s_eff_vel_cmd = velocity_ff_gain * v_ff + vcorr`. Default **1.0** is
behaviour-neutral (current 1:1). `0` = pure feedback; trim in between. Applied live in `od_apply_gains`
(clamped ≥ 0), seeded to 1.0 in `MC_OdStore_LoadDefaults` (so a stray 0/old-flash can't silently kill FF).

**2. The signal generator produces a velocity (its derivative) alongside the position.** `mc_signal_gen`
gains a `vel` field + `MC_SignalGen_Velocity()`: during a RAMP it is the signed ramp `rate`; during DWELL,
a step edge (`rate ≤ 0`), or IDLE it is 0. This makes the generator a reference source that emits
`(value, velocity)` — exactly like the trajectory emits `(position, velocity)`.

**3. Position-tuning uses it.** In the D3 position-tuning branch, `v_ff = MC_SignalGen_Velocity(&s_sig_gen)`
(was 0). The same `s_eff_vel_cmd = velocity_ff_gain * v_ff + vcorr` then applies — so **normal moves and
position tuning share one FF path**. A position *step* (rate 0) still has `v_ff = 0` → pure feedback, the
correct step-response test; a position *ramp* feeds forward its ±`rate` velocity.

## Why this is the tidy shape

The generator and the trajectory are now interchangeable reference sources (`position + velocity`); the
cascade doesn't care which one is driving. No special-casing of FF for tuning — it falls out of the
generator exposing its derivative, and the single `velocity_ff_gain * v_ff` term covers both.

## Verification

- Host test of `mc_signal_gen`: `vel` is ±`rate` during ramps, 0 during dwell/step/idle; integrates to the
  value. `gcc -fsyntax-only` clean.
- On-target (user): position tuning with a ramp — graph `tlm_pos_demand_rad` (0x2510:3) vs `position_actual`
  (0x6064); raise `velocity_ff_gain` and watch following error shrink on the ramp segment.

## Consequences

- Velocity FF is tunable (one gain, both for trajectory moves and position tuning).
- Position tuning now exercises the loop the way a real move does (FF + feedback).
- Default 1.0 keeps existing behaviour; PERSIST so a tuned value survives reboot.

## Files affected

- `../Lightweight_CMC/Interface/mc_if_od.h` (0x2200:4) + `CHANGELOG.md`
- `include/mc_signal_gen.h`, `src/mc_signal_gen.c` (`vel` + accessor)
- `include/mc_od_store.h`, `src/mc_od.c` (field + default 1.0), `src/mc_scheduler.c` (gain + tuning FF)
- `docs/spec/09_control_loops.md`, `requirements.yaml`, `../Lightweight_CMC/Interface/gui/...` (Motor Config row)

## Open questions

- An `accel_ff_gain` for the torque-domain acceleration feedforward — same pattern; deferred unless needed.
