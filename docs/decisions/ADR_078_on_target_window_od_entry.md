# ADR-078: Separate ON_TARGET window OD entry (decouple status tolerance from the control deadband)

- Status: Accepted (supersedes the "window = position deadband" coupling of ADR-076)
- Date: 2026-07-23
- Related: ADR-071 (position deadband), ADR-076 (window = deadband — superseded here), ADR-077
  (ON_TARGET survives drive-disable)

## Context

ADR-076 tied the ON_TARGET / TARGET_REACHED window to `position_deadband_rad`. Live diagnosis on the
brushed OFF-policy axis (192.1.0.101) showed this is wrong in two ways once the deadband is tightened:

1. The **continuous** deadband (ADR-071) parks the axis **at the band edge** — the correction is
   `∝ (|error| − deadband)`, which → 0 as the error approaches the deadband, so the axis settles with
   `|error| ≈ deadband`. With `window = deadband` and a strict `<`, `|error| < deadband` is then
   **never** satisfiable — an axis parked exactly where the deadband leaves it can never report on
   target.
2. With the **OFF idle policy** the drive de-energises before that slow final crawl finishes, so the
   axis freezes **just outside** the deadband (observed: 0.011 rad parked, 0.010 rad deadband → 0.001
   rad outside → reported off, correctly but uselessly).

So the *control* tolerance (how tightly the loop holds — the deadband) and the *status* tolerance (how
close counts as "on the shot") are different quantities and must not share one value.

## Decision

Give ON_TARGET / TARGET_REACHED its own window: **`on_target_window_rad` (0x2200:6, F32 RW PERSIST,
default 0.02)**. The window uses this entry (falling back to `MC_POS_TARGET_WINDOW_RAD` = 0.01 rad when
0); `position_deadband_rad` reverts to being purely the control tolerance.

- Applied at both sites: the cascade `reached` test and the ADR-077 drift check (drop the latch when
  back-driven beyond the window).
- **Must be ≥ the deadband** to be reachable for a deadband-parked / de-energised axis; the 0.02
  default clears a 0.01 deadband with margin for the OFF-policy settle.
- Read via the scheduler config copy `s_on_target_win_rad` (set in `od_apply_gains`), not `g_od`
  directly.

## Consequences

- ON_TARGET reports reliably for an OFF-policy axis parked on the shot, independent of how tight the
  operator makes the positioning deadband.
- Additive OD entry → no `MC_IF_PROTOCOL_VERSION` bump; PERSIST blob additive-safe (an old config
  restores it at the 0.02 default). CHANGELOG v5.13.0.
- Supersedes ADR-076: the window no longer follows the deadband. ADR-071 (deadband as control
  tolerance) and ADR-077 (latch across drive-disable) stand.
- GUI: `on_target_window_rad` sits next to `position_deadband_rad` in Position loop gains, labelled
  status-vs-control.
- Verification build + review only. On 192.1.0.101 after flashing: with default 0.02 and the axis
  parked at 0.011, ON_TARGET / TARGET_REACHED should now assert; jog off and confirm they drop;
  tune `0x2200:6` to taste (keep it ≥ the deadband).
