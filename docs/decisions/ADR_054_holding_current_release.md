# ADR-054: Holding-current release on settle

- Status: Accepted
- Date: 2026-06-30
- Related: ADR-040 (motion envelope), ADR-052 (brushed velocity loop), ADR-011 (PID)

## Context

A self-locking linear actuator doesn't need motor current to hold position once stopped — the
mechanism holds it. Continuously holding zero velocity wastes power and heats the motor. The operator
wants an option to drop the holding current to zero after the axis has settled.

## Decision

New OD entry `0x2300:9 holding_current_a` (F32, RW, PERSIST), **binary** semantics for v1:
- `> 0` (default 1.0): always hold (present behavior).
- `= 0`: **release**. In velocity mode, once the demand is ~0 (commanded zero) **AND** the actual
  velocity is below `MC_HOLD_SETTLED_EPS` (0.1 rad/s) continuously for `MC_HOLD_RELEASE_TICKS`
  (1000 ms @ 1 kHz), the velocity stage forces `s_iq_cmd_published = 0` and **resets the velocity
  controller** (parks the integrator → no drift-windup). It stays released until a non-zero command,
  so a back-drivable axis can't hunt (release → drift → re-engage). Resume is bumpless from reset.

Refinements over the first sketch ("zero the torque 1 s after target velocity 0"): (a) gate on
**actual** velocity settled, not just the target, so it can't cut mid-deceleration; (b) **park the
integrator**, not just the output, to avoid a surge on re-engage; (c) **latch released** until a real
move command to avoid drift hunting. Backend-agnostic (lands at the velocity-loop output, so brushed
and FOC both get it).

## Consequences

- Additive OD entry → no `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected. Persist 384/448 B.
- **Velocity-mode only.** Position-mode holding is a separate concern (with the position/homing work).
- **SAFETY:** releasing holding torque means the axis **drifts if back-drivable**. Only enable (`0`)
  on a self-locking / non-back-drivable mechanism, or where settle is acceptable. Default (`>0`) holds.
- v1 is binary; `>0` is reserved for a future "clamp the held current" (reduced-power hold) refinement.
- Settle threshold + delay are hardcoded (0.1 rad/s, 1 s); promote to OD entries if tuning is needed.

## Bring-up

Set `0x2300:9 = 0`, command a velocity then 0; ~1 s after it stops, watch `vel_iq_cmd_a` / `i_arm`
drop to 0. Command a non-zero velocity → it re-engages smoothly. Confirm the axis holds mechanically.
