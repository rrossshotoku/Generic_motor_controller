# ADR-028: D3 position execution — trajectory → position loop → velocity cascade

## Status

Accepted

## Date

2026-06-22

## Context

`PROFILE_POSITION` was set up by the CMC (SDO targets + `NEW_SETPOINT`) but **not executed** on the
motor — `mc_scheduler.c` explicitly forced the drive safe-off for position mode ("position (not routed
yet) -> safe"). The pieces existed: the trapezoidal trajectory planner (`mc_trajectory.c`, ADR-025) and
the position-controller interface stub (`mc_position_controller.h`). This ADR wires them into the
running cascade so commanded position moves actually run — completing D3 stages 2–5 (ADR-025), and
enabling the PC command page's Position controls.

## Decision

**Position controller (`mc_position_controller.c`)** against the existing stub, per spec 09:
`velocity_correction = PID(position_demand − position_actual)`, **P-only default** (gains `0x2200`
`pos_kp/ki/kd`, `pos_ki = pos_kd = 0` by default). Output clamped to `velocity_correction_limit`;
position error clamped to `following_error_limit` (anti-windup; no following-error fault in this cut).

**Scheduler `PROFILE_POSITION` routing** (medium loop, 1 kHz), home-relative throughout (positions are
relative to `s_home_offset_rad`, matching OD `position_actual` 0x6064):
- On `NEW_SETPOINT` (`ds.new_setpoint_latched`): start a plan — start = current home-relative position,
  target = `0x607A` (home-relative), time = `0x607B`, limits = `0x6081`/`0x6083`/`0x6084`
  (vel/accel/decel). From rest (ADR-025 first cut).
- Each tick: `sp = MC_Trajectory_Update(dt=1ms)`; `vcorr = PositionController_Update(sp.position,
  pos_act)`; **`velocity_demand = sp.velocity_ff + vcorr`** feeds the existing velocity loop
  (`s_eff_vel_cmd`); `sp.acceleration_ff` feeds the torque request's inertia·accel slot (was hardcoded
  0). The drive runs as a normal velocity-class move, so all velocity/torque limits and the
  over-current trip still apply.
- **Hold when there is no active plan.** An inactive/just-failed planner evaluates to `position = 0`
  (`sp.valid == false`); commanding it would slam the axis to home. So when `!sp.valid` the cascade
  **holds a latched position** (captured on entry / the last demand) instead — fixes the "enabling in
  Profile Position drives to 0 before any move is commanded" bug. The plan also **falls back to safe
  default limits** (2 rad/s, 10 rad/s²) when the profile limits arrive as 0, so a move plans rather
  than failing `INVALID_LIMITS` and silently holding.
- **`target_reached`**: trajectory `complete` AND `|position_error| < MC_POS_TARGET_WINDOW_RAD` →
  `statusword` bit `MC_IF_SW_TARGET_REACHED (0x0400)`. Holds at target (trajectory holds final position).

## First-cut scope

- From-rest trapezoidal moves only (ADR-025); replan-from-motion + S-curve later.
- Single fixed target window (`#define`), no OD entry yet; no following-error fault yet.
- Position feedback = estimator mechanical position (observer default), home-relative.

## Verification

- `gcc -fsyntax-only` clean; host test (HAL-free) of the position controller + a trajectory→position
  step (P-only) — converges to target, velocity demand bounded by the profile.
- On-target (user): mode = Profile Position, enable, set target + time, Start move → motor runs the
  trajectory to target; `statusword` TARGET_REACHED sets on arrival.

## Consequences

- Position moves run end-to-end (PC command page Position controls become live).
- Cascade is now trajectory → position → velocity → torque → FOC, all SI, behind the existing arbiter.
- No contract change (uses existing `0x607A/0x607B/0x6081/3/4` + statusword bit).

## Files affected

- `include/mc_position_controller.h` (unchanged interface), `src/mc_position_controller.c` (new)
- `src/mc_scheduler.c` (statics + init + `od_apply_gains` pos gains + `PROFILE_POSITION` cascade + target-reached)
- `docs/spec/09_control_loops.md`, `requirements.yaml`, `tests/`

## Open questions

- Replan-from-motion (bumpless) + S-curve + non-zero start velocity — the next planner pass (ADR-025).
- Target window / following-error limit as OD entries (tuning) — deferred to on-target tuning.
