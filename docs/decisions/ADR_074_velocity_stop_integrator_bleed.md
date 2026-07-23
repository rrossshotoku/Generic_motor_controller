# ADR-074: Velocity-loop stop-integrator bleed (anti-recoil at the end of a jog)

- Status: Accepted
- Date: 2026-07-23
- Related: ADR-012 (velocity controller), ADR-042 (velocity slew ramp), ADR-062 (position-integrated
  jog), ADR-071 (position deadband), ADR-072 (idle policy → CMC / HALT timing)

## Context

Broadcast cameras must not visibly **recoil** at the end of a move — a slight overshoot that settles
is acceptable, but a *reverse twitch* is not. On a jog stop this came from the **velocity loop**, not
the position controller: as the demand goes to 0, the PI's error is negative (`actual > 0`), so the
**integrator winds up negative** commanding brake torque. When the velocity reaches 0 the integrator
is still negative → it keeps braking → drives the velocity **past zero into reverse** → the loop then
recovers → ring. That reverse excursion is the on-camera recoil, and it's integrator-driven: the
proportional term alone (`−kp·velocity`) is a first-order brake that eases to rest **without**
overshoot.

## Decision

Add a **stop-integrator bleed** to the velocity controller: when commanded to stop and actually slow,
fast-unwind the integrator so it can't push the velocity past zero. Three OD params in the velocity
block:

- `vel_stop_bleed_enable` (0x2300:13, U8 RW PERSIST) — master on/off, **0 = off** default.
- `vel_stop_bleed_v_th` (0x2300:11, F32 RW PERSIST) — arm the bleed when `|velocity| < v_th`.
- `vel_stop_bleed_factor` (0x2300:12, F32 RW PERSIST) — unwind speed as a **factor of ki**.

Implementation (`mc_velocity_controller.c`), each medium tick, **before** the PI runs:
```
if (enable && v_th > 0 && factor > 0 && |demand| < 1e-3 && |actual| < v_th)
    integrator -= integrator * clamp(factor·ki·dt, 0, 1)
```
Draining the integral hands the final approach to the proportional brake, which reaches zero without
overshoot → clean stop, no reverse.

**Rate as a factor of `ki`, not an absolute [1/s]:** the per-tick removed fraction is `factor·ki·dt`,
so `factor = 2` bleeds twice as fast (halves the unwind time constant). Tying it to `ki` scales the
bleed with the *windup* timescale — a more aggressive integrator winds up faster and is bled faster —
so the factor stays meaningful across a `ki` retune, and it auto-disables when `ki = 0` (no integral,
nothing to bleed). Explicit `enable` keeps the tuning values (`v_th`, `factor`) set while the feature
is off; defaults are sane (`v_th = 0.5`, `factor = 1`) but inert until enabled.

### Why gate on the demand (and why that's right for shot recall)

The gate is `|demand| ≈ 0` (plus `|actual| < v_th`), **not** the operating mode. That self-scopes it:

- **Jog stop** — the demand *parks* at 0 on release, so the bleed engages firmly through the final
  approach. Clean stop. Covers both direct and position-integrated jog.
- **Shot recall** — the position-cascade velocity demand only *kisses* zero at the target, then goes
  **negative** to correct any overshoot. So the bleed engages only fleetingly and backs off the
  instant the correction demand appears — leaving the velocity loop responsive so the position loop
  can **land on the exact target**. This is deliberate: fully bleeding during a precision landing
  would soften the loop and cause it to miss. Shot-recall *recoil*, if it occurs, is a
  position-domain concern (trajectory/tuning + the ADR-071 deadband + the ADR-072 HALT-timing fix on
  the CMC), not this.

### Why bleed, not clear or freeze (from the design discussion)

- **Hard clear** steps the torque command by the whole integral in one tick → a visible jolt (trades
  a recoil for a jerk). A first-order bleed keeps torque continuous.
- **Freeze** (stop integrating) keeps the already-wound-up value — which *is* the brake that
  overshoots. You must *reduce* it, not just stop adding to it.
- A **sign-opposition** unwind (fast-unwind when the integral opposes the P-term) was considered but
  fires *after* the zero crossing (during decel the integral and P-term are both braking = same
  sign), so it speeds recovery from the recoil rather than preventing it. Proximity-to-stop is the
  trigger that acts in time.

## Consequences

- A jog decelerates to a genuine standstill with no reverse twitch, at the cost of a slightly softer
  final brake (the P-only approach). Tunable via `v_th` (when it arms) and `rate` (how hard).
- Additive non-PDO OD → no `MC_IF_PROTOCOL_VERSION` bump; PERSIST blob ~502/640 B (ADR-070 headroom,
  truncation guard clear). CHANGELOG v5.10.0.
- **Gravity caveat (tilt axes):** the integrator also holds against gravity; a full bleed can let the
  axis sag briefly between "velocity loop lets go" and the CMC's stop→HOLD capture. Mitigations if
  seen: keep `rate` modest, or bleed only when the axis is genuinely near rest; the CMC HALT-timing
  fix (hold captured promptly at standstill) closes the gap. Non-issue on a horizontal pan.
- **Verification is build + review only** — no hardware here. Bench: set `enable = 1`, `v_th` a bit
  above the recoil speed, `factor ≈ 1` (≈ 1×ki), jog and release, confirm the reverse twitch is gone;
  lower `factor` if the stop feels mushy, raise `v_th` (or `factor`) if a twitch still slips through.
