# ADR-045: Jerk-limited S-curve trajectory planner (parallel block)

- **Status:** Accepted
- **Date:** 2026-06-26
- **Relates to:** ADR-025 (trapezoidal planner), ADR-028 (D3 position cascade), ADR-040 (motion envelope), ADR-044 (persist blob)

## Context

The shipped planner (ADR-025, `mc_trajectory.c`) is a trapezoidal profile with a fixed 1/6 : 2/3 : 1/6
time split. It works well and the operator does not want it touched. Its **interface was designed
S-curve-ready**, though: `MC_TrajLimits_t` carries `max_jerk_rad_per_s3`, `MC_TrajSegment_t` carries a
per-segment `jerk` (`a(t) = accel + jerk·t`), `MC_TRAJ_MAX_SEGMENTS = 7`, there is an
`MC_TRAJ_TIME_STRETCHED` status, and the integrator `traj_integrate` already integrates the cubic jerk
term. We want a true **jerk-limited S-curve** as a second, selectable planner.

## Decision

A parallel planning function **`MC_TrajScurve_Plan`** (new module `mc_traj_scurve.c/.h`) fills the
**same** `MC_TrajectoryPlanner_t` with a 7-segment jerk-limited profile; **sampling reuses the existing
`MC_Trajectory_Update` / `MC_Trajectory_Evaluate`** (already jerk-capable). The trapezoidal
`MC_Trajectory_Start` is untouched.

**Fixed jerk, solved peak v/a.** The jerk magnitude `j` is a fixed system constant (OD `0x2600:8`
`max_jerk_rad_s3`); peak acceleration and velocity vary per move within `max_acceleration` /
`max_velocity`. The profile is parameterised by a **single free knob, the peak velocity `V`**:
- `a_peak = min(a_lim, √(j·V))` — *triangular* accel below `V = a_lim²/j` (no const-accel segment),
  *trapezoidal* (clamped at `a_lim`) above;
- accel-phase distance `d_acc = (V/2)·t_acc` (the velocity ramp is antisymmetric, so its mean is `V/2`);
- total time `T(V) = t_acc(V) + D/V`, **monotonic decreasing in `V`**.

**Time = goal with limit-priority.** Solve `T(V) = requested_time` for `V` by **bisection** (T is
monotonic → robust, and avoids fragile piecewise closed-form cubics); clamp `V` to `v_max` and to the
distance-limited `V` (where the cruise vanishes); `a_peak ≤ a_lim` by construction. If the requested
time is shorter than the fastest feasible (`T_min` at `V_ceil`), **extend the time to `T_min`**
(`MC_TRAJ_TIME_STRETCHED`) rather than breach a limit. Any time `≥ T_min` is achievable; you can always
go slower (lower `V`), never faster than `T_min`.

**Rest-to-rest only** (target velocity/accel = 0), matching the requested scope and the trapezoidal cut.

**Selection** via a motor-owned flag (OD `0x2600:9` `traj_use_scurve`, PERSIST): the scheduler calls
`MC_TrajScurve_Plan` when set, else `MC_Trajectory_Start`. Both feed the same sampler + position cascade.

## Consequences

- New module `mc_traj_scurve.c/.h`; `mc_trajectory.c` unchanged.
- New OD entries `0x2600:8 max_jerk_rad_s3` (F32) and `0x2600:9 traj_use_scurve` (U8), both RW PERSIST
  motor-owned, in the motion-limits block. Additive non-PDO → **no `MC_IF_PROTOCOL_VERSION` bump**;
  CHANGELOG; GUI exposes them in the motion-limits config group. (+13 B persist, well within the 448 B
  blob — ADR-044.)
- Host test `tests/test_traj_scurve.c`: lands at target at rest, stays within v/a limits, jerk = ±j,
  hits the requested time when feasible, stretches when too short, monotonic (no overshoot).
- The bisection runs **once per move-plan in the medium loop** (not the fast loop) — ~60 iterations,
  negligible.

## Rejected

- **Filling the jerk segments inside `mc_trajectory.c`** (its header anticipated this): would modify the
  proven trapezoidal block, which the operator explicitly wanted left alone.
- **Closed-form solve:** the time equation is piecewise (triangular/trapezoidal × cruise/no-cruise) with
  cubic terms; bisection on the monotonic `T(V)` is simpler and robust.
- **Per-move jerk:** rejected by requirement — one fixed jerk gives consistent smoothness across moves.
- **Arbitrary start velocity / on-the-fly retarget:** out of scope (rest-to-rest only) for now.
