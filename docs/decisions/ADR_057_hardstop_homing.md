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
  The stop is detected by `|armature current|
  > home_current_a` for `MC_HOME_DWELL_MS` (**10 ms**) **OR the OC trip firing** — a hard stop spikes the
  current past the OC limit before a longer dwell would confirm, and the trip *blocks* the drive, so it
  must count as the stop signal. It captures the current position as the encoder zero (`s_home_offset_rad
  = position`, like set-mech-zero, auto-saved), **clears the OC trip if that was the trigger** (it was the
  intended stop signal → axis stays usable), and stops → **DONE**. A
  `MC_HOME_TIMEOUT_MS` (30 s) safety abort covers "current never trips" → **FAILED**. Zero = the stop
  (no back-off). `home_command = 0` aborts / clears a latched DONE/FAILED.
- Uses the velocity loop, so it needs the quad feedback (`0x2500:8`). Push force is bounded by the
  velocity current limit (`0x2300:4`); set `home_current` below it so it can trip.

OD (`0x2700` calibration block): `home_velocity_rad_s` (:6, F32 PERSIST), `home_current_a` (:7, F32
PERSIST), `home_command` (:8, U8 RW), `home_status` (:9, U8 RO → `MC_IF_HOME_IDLE/RUNNING/DONE/FAILED`).
PC tool: a **"Home to end stop"** section (velocity + current fields, Home/Stop, live status),
disabled on the FOC backend (homing is for incremental encoders).

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
