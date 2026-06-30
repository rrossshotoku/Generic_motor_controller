# ADR-056: Abandon the position trajectory on leaving position mode

- Status: Accepted
- Date: 2026-06-30
- Related: ADR-028 (position cascade), ADR-035 (HALT hold), the joystick-trim flow

## Context

When a position move completes, the planner keeps `active = true` (only HALT cleared it), so
`MC_Trajectory_Evaluate` keeps returning a **valid** setpoint at the old target. If the axis then
leaves position mode (e.g. a velocity-mode joystick **trim** to a new spot) and returns, the position
cascade re-uses that stale trajectory: it overwrites the freshly-latched current position with the
old target and drives back — "moves us back" instead of holding where the trim left it.

## Decision

On leaving position mode (the D3 cascade's `else`, on the transition out), abandon the plan:
`if (s_pos_on) { s_traj.active = false; }`. An inactive planner makes `MC_Trajectory_Evaluate` return
`valid = false`, so on re-entry the cascade falls back to holding `s_pos_hold_rad` — which the entry
block re-latches to the **current** position. So trimming with the joystick and returning to position
mode **holds the trimmed position**. A fresh move still needs a new setpoint (NEW_SETPOINT starts a
new plan). The normal completed-move hold (never left position mode) is unchanged — `active` stays
true and it holds at the target exactly.

**On-target reporting (build 74):** that abandoned/manual hold is *not* a commanded target, so it must
not report TARGET_REACHED / on-shot. The cascade now tracks `at_cmd_target` — true only for a genuine
completed move (an active plan), false in the no-active-plan hold branch — and `reached = complete &&
at_cmd_target && |perr| small`. So after a joystick trim, the motor stops reporting on-target/on-shot
to the CMC (the CMC's target was the recalled position, not the manual trim); a fresh move that
completes sets it again. (During the trim itself the axis is in velocity mode, where the statusword is
rebuilt without TARGET_REACHED anyway, so that case was already clean.)

## Consequences

- Motor-only, no OD/contract change, no `MC_IF_PROTOCOL_VERSION` bump.
- **CMC caveat:** the motor still obeys an explicit command — if the CMC re-issues the old target with
  NEW_SETPOINT on returning to position mode, it moves back (correct CiA-402). The CMC should return
  to position mode **without re-asserting the old target** (or set target = current first). With this
  fix, "just switch the mode back" holds the current position.
- Also covers a mid-move interruption (OC trip, disable): the move is abandoned, not auto-resumed on
  re-entry — safer (re-command explicitly).
