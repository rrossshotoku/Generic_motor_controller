# ADR-040: Motor owns and enforces the motion envelope; CMC requests within it

- **Status:** Accepted
- **Date:** 2026-06-26
- **Relates to:** ADR-013/014 (shared contract + limits), ADR-028 (D3 position cascade), ADR-039 (brushed backend; homing OPEN-E)

## Context

The axis motion limits were CMC-owned and mirrored into the motor:

- `axis_velocity_limit` (0x3030) / `axis_accel_limit` (0x3033) are SDO-written by the CMC's
  axis_manager into the motor's `profile_velocity` / `profile_acceleration` / `deceleration`
  (0x6081/3/4), which the trajectory planner reads.
- `axis_position_limit_lo/hi` (0x3031/2) are CMC-only.

This conflates two genuinely different concepts under one CMC-owned number:

1. the **hardware/safety envelope** — the absolute maximum the motor + mechanics can safely
   sustain (max velocity, max acceleration, the position/stroke travel); and
2. the **per-move operational request** — how fast to run a given move.

The motor held no independent ceiling: it executed whatever profile the CMC sent. A CMC fault,
a mis-scaled command, or operator error could drive the actuator past its safe envelope, and the
limit value lived in two places (CMC + the mirrored motor copy).

## Decision

Split the two and assign ownership by role.

- **The motor owns and ENFORCES the hardware/safety envelope.** New motor-owned, persisted,
  per-board limits — `max_velocity_rad_s` (0x2600:4), `max_accel_rad_s2` (0x2600:5) — and, in a
  later phase, the position/stroke limits. The motor clamps **every** trajectory and the velocity
  demand to this envelope **regardless of the source** (CMC cyclic, operator, signal generator,
  watch window). Nothing upstream can command past the safe limits; the motor is the safety
  authority.
- **The CMC owns the per-move operational request.** `profile_velocity` / `profile_acceleration`
  (0x6081/3/4) remain the CMC's per-move request, **bounded** by the motor envelope. The CMC keeps
  full control of how fast each move runs, within the ceiling. They stop being a naked mirror; they
  are "the request, clamped."

Enforcement (this phase — velocity + acceleration):

- Trajectory planning clamps the requested profile to the envelope:
  `max_velocity = min(profile_velocity, max_velocity_rad_s)`, similarly for accel/decel.
- The final velocity demand `s_eff_vel_cmd` is clamped to `±max_velocity_rad_s` at the
  velocity-loop input (`vdem`) — one place that covers **all** modes (position cascade, direct
  velocity, signal generator).
- A ceiling of **0 = disabled** (no clamp): the safe migration default, so existing behaviour is
  unchanged until the operator sets the real per-board envelope.

## Consequences

- New motor OD entries `0x2600:4 max_velocity_rad_s`, `0x2600:5 max_accel_rad_s2` (F32 RW PERSIST,
  motor-owned, default 0 = disabled). Additive, non-PDO → **no `MC_IF_PROTOCOL_VERSION` bump**.
  CHANGELOG [4.5.0]; GUI Motor Config exposes them.
- `profile_velocity` / `accel` keep their meaning (the CMC's request) but are now clamped — no
  contract change to those entries.
- **Motor-owned soft position limits** (`0x2600:6/7 pos_limit_lo/hi_rad`, home-relative, **manually
  set** like the mech zero — *not* coupled to homing): the motor clamps move targets, stops the
  velocity demand at a limit, and sets the `AT_LIMIT_LO/HI` movement-status bits. `lo >= hi` = disabled.
  Homing remains a separate, optional way to populate them later. The CMC's `0x3031/2` are now
  redundant for the motor axis — harmless if left (advisory); retiring them is a later CMC-author
  cleanup (`Interface/REQUESTS.md` when convenient), not a blocker.
- Safety becomes structural — a clamp the motor always applies — rather than a trust contract with
  the CMC.

## Rejected

- **CMC keeps owning the limits (status quo).** A CMC-owned limit is advisory; the motor trusts it.
  For a safety limit you want a guarantee the motor enforces itself.
- **A single motor-owned number (no request/ceiling split).** Loses the operator's per-move speed
  choice; you still want a separate request, so it collapses back into this design anyway.
