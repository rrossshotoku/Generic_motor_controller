# ADR-025: Trajectory planner — trapezoidal first cut (D3 stage 1)

## Status

Accepted

## Date

2026-06-22

## Context

D3 (position control) per `docs/spec/08_trajectory_planner.md` + `09_control_loops.md`. The planner
interface was **pre-defined** in `include/mc_trajectory.h` (shaped for the end-state jerk-limited
S-curve: `MC_TrajRequest_t`/`MC_TrajLimits_t`/`MC_MotionSetpoint_t`/`MC_TrajPlanInfo_t` and
`Init`/`Start`/`Replan`/`Update`/`Evaluate`/`GetPlanInfo`, with a 7-segment internal model).

Decision (asked): **start with a trapezoidal profile**, fixed **1/6 : 2/3 : 1/6** (accel : cruise :
decel) time split, with `accel = velocity = 1` defaults; the S-curve and jerk limit come later.

## Decision

Implement against the **existing public interface unchanged**; only the internal segment grew an
`accel` field (`MC_TrajSegment_t` → `{duration, accel, jerk}`), because a trapezoid's *step* in
acceleration can't be represented by `{duration, jerk}` alone — and the general `a(t) = accel +
jerk·t` segment covers the trapezoid (jerk 0) **and** the future S-curve (jerk ramps), so the model
generalises.

Trapezoidal math (from rest, target vel/accel 0, fixed split `t_a = t_d = T/6`, `t_c = 2T/3`):
`D = v_cruise·5T/6` ⇒ `v_cruise = 1.2·D/T`, `a = 7.2·D/T²`; minimum feasible time
`T_min = max(1.2·D/v_max, √(7.2·D/a_lim))`. A `requested_time` ≥ `T_min` is honoured (gentler move);
shorter is **stretched** to `T_min` with `time_stretched` + `MC_TRAJ_TIME_STRETCHED`. Stored as three
constant-accel segments; `Evaluate` integrates them.

**First-cut limitations** (spec 08 allows a conservative first implementation):
- Plans **from rest** — start velocity/accel are accepted in the API but treated as 0. (Replan
  carries the start *position*; bumpless replan-from-motion arrives with the S-curve.)
- Non-zero **target** velocity/accel → `MC_TRAJ_ERR_UNSUPPORTED_BOUNDARY`.
- `max_jerk` accepted but unused. Symmetric profile → uses the tighter of accel/decel limits.

## Verification

Host test (`gcc`, HAL-free) over 6 moves — ASAP, timed-feasible, timed-too-short (stretched), large
(both limits binding), negative direction, short:
- every sample **bounded**: peak velocity ≤ `v_max`, peak accel ≤ `a_max`;
- **end position exact** (err < 1e-6 rad); `complete` set; time-stretch reported correctly;
- e.g. 1 rad ASAP → 2.683 s, peak 0.447 rad/s, accel 1.0; 5 rad ASAP → 6.0 s, peak 1.0/accel 1.0
  (both limits binding). `gcc -fsyntax-only` clean.

## Consequences

Stage 1 of D3 done and proven offline. Remaining D3 stages (next):
2. `mc_position_controller` — thin PID wrapper (P-only default, gains `0x2200`), per spec 09.
3. Scheduler integration — `PROFILE_POSITION` + `NEW_SETPOINT` (`ds.new_setpoint_latched`) triggers a
   plan (target `0x607A` home-relative, time `0x607B`, limits `0x6081/3/4`); each 1 kHz tick:
   trajectory → position loop → `velocity_demand` (vel FF + correction) → velocity loop; `accel_ff`
   → the torque request's inertia·accel slot.
4. Target-reached → statusword `TARGET_REACHED` (0x6041 bit 10); hold at target.
5. On-target tuning.

Then the S-curve upgrade (fill the jerk segments; non-zero start velocity; jerk-limit OD entry).

## Files affected

- include/mc_trajectory.h (segment `accel` field; trapezoidal-first `@brief`)
- src/mc_trajectory.c (new)
- docs/decisions/ADR_025_trajectory_planner_trapezoidal.md, docs/decisions/ADR_000_decision_log.md, docs/spec/08_trajectory_planner.md

## Open questions

- S-curve (jerk segments) + non-zero start velocity + a jerk-limit OD entry — the next planner pass.
- Position-controller + scheduler integration (D3 stages 2–5).
