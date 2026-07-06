# ADR-057: PC-triggered hard-stop homing / encoder zeroing

- Status: Accepted
- Date: 2026-07-01
- Related: ADR-050/052 (quad encoder), ADR-022 (mech zero), ADR-040/043 (soft limits)

## Context

An incremental (quad) encoder has no absolute position at power-on — the count starts at 0 wherever
the axis happens to be. Absolute position mode + soft limits need a zero reference; for a linear
actuator the natural reference is a hard end stop.

## Decision

A **PC-triggered drive-to-hard-stop** homing routine (auto-home-at-startup deferred — the same
routine, a future second trigger):
- On `home_command = 1` (and not preempted by watch-inject / dq-test), the motor drives **velocity
  mode** at `home_velocity_rad_s` (signed; toward the stop), **ramped through the accel limiter**
  (`vel_slew_limit`, `0x2300:6/7/8`, ADR-042 — not stepped; reset from the current velocity on start).
  The stop is detected by **movement going negligible** (build 80): once the axis has actually moved
  (`|velocity| > MC_HOME_STILL_EPS` = 0.01 rad/s — *armed*, so the initial ramp from rest can't be mistaken
  for the stop), a run of `|velocity| < MC_HOME_STILL_EPS` lasting `MC_HOME_STILL_MS` (**1000 ms**) **OR the
  OC trip firing** confirms the end stop. On finding it (build 86) the motor **captures the encoder zero
  right there — at the hard stop** (`s_home_offset_rad = position`; the stop is the datum, position 0),
  **clears the OC trip if that was the trigger**, then **backs off** — drives the OPPOSITE direction
  (ramped, same `vel_slew_limit`) for `MC_HOME_BACKOFF_MS` (**1000 ms**) — auto-saves and stops → **DONE**.
  The axis is left backed off *clear of* the stop; when position mode re-engages, the hold-on-enable
  latches the hold at the current (backed-off) position (`s_pos_hold_rad = p_act`), so it holds off the
  stop rather than driving 0 back into it (which would fight the hard stop). Any *stale* OC trip is **cleared
  at start** so a latched trip can't instantly "find" the stop. `MC_HOME_TIMEOUT_MS` (30 s) safety abort
  (checked during the approach) covers "never settles / trips" → **FAILED**. `home_command = 0` aborts /
  clears a latched DONE/FAILED.
- Uses the velocity loop, so it needs the quad feedback (`0x2500:8`). Push force is bounded by the
  velocity current limit (`0x2300:4`); the OC trip (`current_trip_a`, `0x2600:2`) is the current-based
  backstop.

OD (`0x2700` calibration block): `home_velocity_rad_s` (:6, F32 PERSIST), `home_command` (:8, U8 RW),
`home_status` (:9, U8 RO → `MC_IF_HOME_IDLE/RUNNING/DONE/FAILED`). `home_current_a` (:7) is **deprecated**
(build 80) — the current dwell was replaced by no-movement detection; the entry is kept unused to avoid an
OD-layout / `MC_IF_PROTOCOL_VERSION` change. PC tool: a **"Home to end stop"** section (approach-velocity
field, Home/Stop, live status), disabled on the FOC backend (homing is for incremental encoders).

## Position recalls gated on homed (build 77)

Until an incremental axis has a zero, an absolute position recall would be relative to the meaningless
power-on count. So a **non-persisted `s_homed` flag** (false at every boot — must re-home each
power-up) gates PROFILE_POSITION moves: a NEW_SETPOINT only starts a trajectory when the encoder is
**absolute** (SSI — always OK) **or** `s_homed` (incremental, zeroed this session). Set by homing DONE
and by set-mech-zero. While an incremental axis is un-homed, `0x2600:1 fault_flags` bit
`MC_IF_FAULT_NOT_HOMED` is raised so the CMC knows to home first (and not command recalls). Velocity
mode / jogging / homing itself are unaffected — only position recalls are blocked.

## Consequences

- Additive OD (non-PDO) → no `MC_IF_PROTOCOL_VERSION` bump; the CMC forwards it like any motor entry.
  Persist 397/448 B. Announced in CHANGELOG.
- **Bypasses the CMC while running** (like dq-test) — `axis_manager` must be OFF. The 30 s timeout, the
  velocity current limit, and the OC trip bound a wrong-sign / no-stop attempt; the operator can abort.
- Absolute position mode + soft limits become usable on the incremental axis once homed.
- Deferred: auto-home at startup (same routine, boot trigger); a back-off after zeroing; FOC-on-quad
  (the stall current is the brushed armature current today).
