# ADR-062: Position-integrated jog (parallel to velocity jog)

- Status: Accepted
- Date: 2026-07-07
- Related: ADR-042 (velocity-demand ramp), ADR-028/031 (position cascade), ADR-054/056 (hold), the following-error guard

## Context

Joystick jogging runs in PROFILE_VELOCITY: the CMC streams a `velocity_setpoint`, the motor runs the
velocity loop directly. So there is **no position demand** — the position **following-error** protection
(and the soft-limit clamp, and a stiff position hold) do **not** apply while jogging. The mode a human
drives by hand is the one with the least protection; only the OC trip catches a jam, and late.

## Decision

Add a **parallel** position-integrated jog, selectable per axis, defaulting to the current behavior:
- New OD **`jog_position_mode`** (`0x2300:10`, U8 RW PERSIST, motor-owned): `0` = direct velocity
  (default, unchanged); `1` = position-integrated.
- The fork lives at the **PROFILE_VELOCITY handler**. When `= 1`, the motor integrates the *ramped*
  velocity setpoint into the existing position hold target (`s_pos_hold_rad += vel·dt`) and runs the
  position cascade (`s_eff_position_mode = true`, `s_traj.active = false`); the reference is **leashed**
  to the actual (`MC_JOG_LEASH_RAD`, 1 rad — can't run away from a stuck axis) and clamped to the
  soft-limit band. D3's hold-on-enable latches `s_pos_hold_rad = actual` on entry; the jog integrates
  from there. Release the stick → the reference stops → the loop holds that position.
- The CMC / contract / cyclic PDO are **unchanged** (the CMC still streams `velocity_setpoint`); the fork
  is entirely motor-side. From the wire it's still PROFILE_VELOCITY.

## Consequences

- **Reuses** the position cascade + hold + soft limits + the following-error guard — no second control
  stack; the new path is a selector-gated branch at one seam, and the old path is untouched.
- Position-integrated jog inherits **following-error / soft-limit / stiff-hold** behaviour for free; the
  feel is preserved (rate = stick deflection, carried by the velocity FF).
- Additive OD (non-PDO) → no `MC_IF_PROTOCOL_VERSION` bump. Default `0` → no behaviour change until flipped.
- **Follow-up:** the position following-error *fault* itself (a `fault_flags` bit + tunable
  window/timeout) is not yet wired — the guard clamps but doesn't fault. The leash currently uses a fixed
  `MC_JOG_LEASH_RAD`; when the fault lands, both should key off the tunable following-error window.
- Evaluate-then-remove: A/B via the flag; removing either path = deleting one branch + the flag.
