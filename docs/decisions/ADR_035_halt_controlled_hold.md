# ADR-035: HALT — controlled hold (stay enabled, hold position)

- **Status:** Accepted
- **Date:** 2026-06-25
- **Related:** ADR-018 (mode manager), ADR-028 (D3 position cascade); contract `CW_HALT` (0x0100)

## Context

The only stop wired was **quick-stop**, which clears the CMC's `enable_latch` → drive disabled → needs an
explicit re-enable. The contract already defines `CW_HALT` (0x0100) as *"controlled stop, hold position"*
and the CMC's `HOLD` op-mode sends `QUICK_STOP | ENABLE | HALT` expecting the motor to hold — but the
motor's mode manager treated HALT as **disable** (`operation_enabled = false` → loops off → coast), so
HOLD didn't actually hold. (User chose **Option 1**: pin-and-hold, no profiled decel.)

## Decision

Implement HALT as a **controlled hold**:

- **Mode manager** (`mc_mode_manager.c`): HALT (with ENABLE, no quick-stop, no fault) → `enabled = true`,
  `active_mode = MC_MODE_POSITION_HOLD` (the previously-unused enum value); statusword keeps **ENABLED**.
  Holds regardless of `mode_of_operation`; resume by clearing HALT.
- **Scheduler** (`mc_scheduler.c`): `MC_MODE_POSITION_HOLD` → `s_eff_drive` stays true,
  `s_eff_position_mode = true`, `s_eff_halt = true`. On the HALT **rising edge**: abandon any in-progress
  move (`s_traj.active = false`), **capture the current position** into `s_pos_hold_rad`, reset the
  position controller. D3 holds at `s_pos_hold_rad` (pure feedback, no FF). New setpoints are ignored
  during HALT (the trajectory-start path lives only under `MC_MODE_PROFILE_POSITION`). Resume → trajectory
  inactive → `MC_Trajectory_Evaluate` returns `valid=false` → D3 holds at `s_pos_hold_rad` until a fresh
  `NEW_SETPOINT`.

**Option 1 (pin-and-hold), no profiled deceleration:** if the axis is moving when HALT engages, the
position loop brings it back to the captured point (bounded by the velocity-loop current limit) — possibly
with some overshoot. A profiled decel (via `profile_deceleration` 0x6084 / `quick_stop_deceleration`
0x6085) is **Option 2, deferred**.

**Advantage over quick-stop:** the drive stays **ENABLED** (the CMC keeps `enable_latch` for the HOLD
op-mode), so resume needs **no explicit re-enable** — just clear HALT (switch the op-mode back).

No contract change (`CW_HALT`, `MC_MODE_POSITION_HOLD` already exist) → **no version bump**. Motor-only —
the CMC's existing `HOLD` op-mode now does what its README claims ("Motor MCU holds current position").

## Consequences

- HALT/HOLD is now a usable gentle stop: holds position, drive stays live, resume without re-enabling.
- Not a profiled decel (Option 1) — abrupt-ish from speed; bounded by the current limit. Option 2 if needed.
- The over-current trip and velocity/current limits still apply (the hold runs through the normal cascade).

## Files

`src/mc_mode_manager.c`, `src/mc_scheduler.c`; docs (`ADR_000_decision_log.md`, `requirements.yaml`,
`docs/spec/09_control_loops.md`).
