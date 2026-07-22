# ADR-071: Position-error deadband (park zone) in the position controller

- Status: Accepted
- Date: 2026-07-22
- Related: ADR-028 (position controller), ADR-054 (holding-current release), ADR-069 (current
  demand limit), ADR-031 (velocity feedforward)

## Context

The position loop (`mc_position_controller.c`) applied a proportional correction to **any** nonzero
position error — there was no dead zone. Combined with `holding_enable = 1` (the loop actively holds
zero velocity at rest) and the unreliable low-level armature-current feedback at ~50 % H-bridge duty,
the axis never cleanly "parks": it keeps issuing tiny corrections around the target, which manifests
as low-level dribble/creep/hunting at the hold point. A position-error deadband is the standard fix.

## Decision

Add `position_deadband_rad` (0x2200:5, F32 RW PERSIST, **0 = disabled** default): the position loop
issues **no correction within ±deadband of the target**, so the axis parks instead of hunting.

- **Where:** in `MC_PositionController_Update`, applied to the error that drives the loop, **before**
  the following-error clamp. The raw error is still reported in `position_error_rad` (telemetry) —
  only the loop's acted-on error is deadbanded.
- **Continuous form (chosen):** inside ±db → 0; outside → `e ∓ db` (subtract the band). The
  correction therefore reaches 0 **smoothly at the band edge** rather than stepping to `P·db`. This
  matters because a hard pass-through deadband's boundary step is itself a good way to *excite* an
  edge limit cycle — the opposite of the intent. Steady state still parks anywhere within ±db (for
  the axis to be at rest the position loop's velocity demand must be ~0 → effective error ~0 →
  `|raw error| ≤ db`), so the "park zone" is identical; only the transient at the edge is smoother.
- **Affects the hold, not the move:** during a trajectory the demand is dominated by the velocity
  feedforward (ADR-031); the deadband only nulls the small *correction* term, so following during
  motion is unaffected in practice (db ~ mrad vs moves ~ rad).
- **No cross-validation, but a documented rule:** keep `deadband < target-reached window`
  (`MC_POS_TARGET_WINDOW_RAD` = 0.01 rad) so TARGET_REACHED is still reported when parked. Not
  enforced — the operator sets it (same philosophy as the current-demand-limit vs OC-trip).

## Consequences

- Eliminates hold-point hunting/creep at the cost of up to ±db of steady-state position tolerance —
  the intended trade. Pairs naturally with the low-level current-feedback limitation on the brushed
  backend: the deadband stops the loop from chasing errors the sensor can't even resolve.
- Interacts cleanly with `holding_enable`: with `= 1` the axis still holds, but only outside the
  band; with `= 0` the release logic is unchanged (deadband just decides when "commanded zero" holds
  produce no correction before release).
- Additive non-PDO OD entry → no `MC_IF_PROTOCOL_VERSION` bump; PERSIST blob now 481/640 B (ADR-070
  headroom), so it persists (the recurrence guard confirms no truncation).
- Derivative-on-measurement is unchanged (D acts on actual position, not the deadbanded error), so
  with a nonzero `pos_kd` the loop still damps motion inside the band — desirable, not a driving term.
- **Follow-up:** consider a small hysteresis if any residual edge dither is seen on a very
  back-drivable axis; expose the effective (deadbanded) error in telemetry if useful for tuning.
