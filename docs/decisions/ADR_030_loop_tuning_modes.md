# ADR-030: Loop-tuning modes — motor-side test-signal overlay + shared signal generator

## Status

Accepted

## Date

2026-06-24

## Context

Tuning the velocity and position loops needs a **clean, repeatable reference signal** (step / ramp /
pulse / continuous) fed at the right point of the cascade. The earlier GUI-side pulse (writes
`axis_target_velocity` over UDP at ~20 Hz) is too coarse for a real ramp, and bolting a generator onto
`PROFILE_VELOCITY` as an override conflates test signals with normal operation. We want the signal
generated **on the motor** (1 kHz, precise) and injected explicitly at the loop under test.

Constraints:
- The signal must be motor-generated (UDP can't produce a clean ramp).
- **No CMC change** (the `axis_manager` is the user's other project) — so the tuning selector must be a
  **motor-owned** OD concept, not a new CiA-402 `0x6060` mode (which the CMC would have to route) and not
  a direct `0x6060` write from the GUI (that desyncs the CMC's cached mode — a failure we already hit).
- Must reuse the existing **enable + safety** path (controlword/mode-manager/over-current trip), and keep
  the CMC's drive-state in sync — so a tuning state must **not** self-enable the drive behind the CMC.

## Decision

Add a **motor-owned `test_mode` overlay** (0x2910 block) that, while the matching operational mode is
**enabled**, redirects that loop's reference to an on-motor **signal generator**. It is a commissioning
concept (the OD/GUI-driven evolution of the watch-window `inject_enable` path), distinct from the
operational CiA-402 modes.

- **`test_mode` (0x2910:1):** `0 = off/normal`, `1 = velocity-tuning`, `2 = position-tuning`.
- **Velocity-tuning** (requires the drive enabled in `PROFILE_VELOCITY`): the generator output replaces
  `s_eff_vel_cmd` (the velocity-loop demand), absolute from 0. Tests the velocity loop in isolation.
- **Position-tuning** (requires the drive enabled in `PROFILE_POSITION`): the generator output, added to
  the position captured on entry, replaces the **position demand** into the position controller,
  **bypassing the trajectory planner** (velocity-FF = 0). Tests the position loop in isolation.
- It is an **overlay**, not a self-enabling mode: enable + safety + CMC drive-state come from the normal
  path; `test_mode` only swaps the reference source. The arbiter gains a test branch:
  `if (test_mode active) redirect the loop reference; else if (inject_enable) commissioning; else remote`.

**Shared signal generator (`mc_signal_gen`)** — one pure, HAL-free, host-testable module reused for both
domains (a velocity ramp and a position ramp are the same primitive: ramp a value toward a target at a
rate, dwell, return). State machine `IDLE → RAMP → DWELL(peak) → RAMP(→0) → IDLE`; `rate = 0` ⇒
instantaneous edge (step); `continuous` ⇒ dwell at 0 for `pause_s` (the inter-pulse pause) then flip the
peak sign and repeat (ping-pong train). Units are the caller's
(velocity tuning: amplitude rad/s, rate rad/s²; position tuning: amplitude rad, rate rad/s).
When `max_accel > 0` the RAMP is **acceleration-limited** (trapezoidal velocity; cruise `rate`, accel
`max_accel`) instead of a constant-`rate` linear ramp — see ADR-032; used for position tuning only.

**Trigger** via OD (`test_trigger`, slow-loop edge → request flag → medium-loop `Start`), consistent with
`cal_command`. Clearing `test_mode` (or Stop) ramps the generator back to 0 (**bumpless**). The run goes
through the loops, so the current clamp + over-current trip still apply.

## OD block (0x2910, motor-owned, additive)

| Sub | Name | Type | Acc | Meaning |
|----|------|------|-----|---------|
| 1 | `test_mode` | U8 | RW | 0 off / 1 velocity-tuning / 2 position-tuning |
| 2 | `test_amplitude` | F32 | RW | peak (rad/s velocity, rad position) |
| 3 | `test_rate` | F32 | RW | ramp rate (rad/s² velocity, rad/s position); 0 = step |
| 4 | `test_dwell_s` | F32 | RW | hold at peak [s] |
| 5 | `test_continuous` | U8 | RW | 0 one-shot / 1 alternating train |
| 6 | `test_trigger` | U16 | RW | write 1 to fire |
| 7 | `test_active` | U8 | RO | 1 while generating |
| 8 | `test_signal` | F32 | RO **PDO** | raw generator output, for graphing the test signal (map into 0x2A00) |
| 9 | `test_pause_s` | F32 | RW | inter-pulse pause at 0 [s] (continuous mode; independent of `dwell`, 0 = none). Added 2026-06-24 |
| 10 | `test_max_accel` | F32 | RW | accel limit [rad/s²] for the **position-tuning** profile (trapezoidal velocity); 0 = off (linear ramp). ADR-032. Added 2026-06-24 |

## Workflow (operator)

1. Set the operational mode (`Profile Velocity` / `Profile Position`) and **Enable** (normal path / CMC).
2. Arm `test_mode` (1 or 2) + amplitude / rate / dwell / continuous (the GUI Tuning section also sets the
   matching op-mode for you).
3. **Fire** → the generator drives the loop reference. Graph: velocity tuning → `tlm_vel_demand_rad_s`
   (0x2310:1) vs `tlm_vel_actual_rad_s` (0x2310:2); position tuning → `tlm_pos_demand_rad` (0x2510:3,
   the absolute demand = entry + signal) vs `position_actual` (0x6064). `test_signal` (0x2910:8) is the
   raw generator output in either mode. (For position tuning `test_amplitude` is the peak offset, in
   rad, from the position captured at Fire.)
4. Stop / `test_mode = 0` → bumpless return to 0 (and the streamed setpoint resumes).

## Alternatives considered

- **GUI-side ramp:** UDP-paced staircase — fine for a step edge, not a ramp. Rejected (the reason we're here).
- **New CiA-402 `0x6060` tuning modes:** the CMC would have to route them (a CMC change), or the GUI writes
  `0x6060` directly and desyncs the CMC's cache. Rejected — keep operational modes clean and CMC-owned.
- **Self-enabling tuning state:** would run the drive without the CMC's ENABLE, desyncing `axis_state` and
  bypassing arbitration. Rejected — the overlay layers on the normal enable instead.
- **Reuse the trajectory planner for velocity ramps:** its API is position-targeted; reverse-engineering a
  position move to get a velocity profile is awkward. A dedicated generator is cleaner.

## Verification

- `gcc -fsyntax-only` clean; host test of `mc_signal_gen` (step, ramped pulse, continuous-alternating,
  signed, bumpless stop).
- On-target (user): velocity-tuning step/ramp into the velocity loop, watch demand vs actual; position-
  tuning step around the entry position; over-current trip still protects; clearing the mode returns to 0.

## Consequences

- Clean, motor-generated test signals injected at the right loop, with explicit state — no override
  ambiguity, no operational-mode pollution, no CMC change.
- The signal generator is reusable (a future current-tuning mode slots in the same way).
- One small branch added to the D3 stage (trajectory vs generated position demand); the bulk of the math
  lives in the pure module, keeping `mc_scheduler.c` from growing further.

## Files affected

- `../Lightweight_CMC/Interface/mc_if_od.h` (0x2910 block) + `CHANGELOG.md`
- `include/mc_signal_gen.h`, `src/mc_signal_gen.c` (new); `include/mc_od_store.h`, `include/mc_debug.h`
- `src/mc_scheduler.c` (arbiter test branch + per-loop injection + trigger + mirror)
- `docs/spec/09_control_loops.md`, `requirements.yaml`
- `../Lightweight_CMC/Interface/gui/mc_gui/main_window.py` (Tuning section)

## Open questions

- Sine/chirp signals for closed-loop frequency response (Bode) — a later generator shape if needed.
- Whether velocity-tuning should support a non-zero bias (tune around a running speed) — deferred; from-rest first.
