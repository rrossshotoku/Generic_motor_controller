# Staged Bring-up & Test Plan

How we incrementally build and verify the framework on real hardware over SWD
(per ADR-005). Each stage adds one capability, stays compileable, and is checked in the
debugger **live watch window** before moving on.

## Per-stage workflow

1. Implement one module/capability (clean-room from the spec, ADR-002).
2. If it changes architecture/interfaces, write the ADR + doc updates first (per
   `CODING_AGENT_DOCUMENTATION_UPDATE_PROCESS.md`).
3. Build in STM32CubeIDE (Debug, -Og), flash over SWD.
4. Add the stage's variables to the live watch window; drive inputs via `g_mc_inject`.
5. Check pass criteria; compare against `../bldc_axis_controller` where applicable.
6. Commit + write a session summary.

## Global safety rails

- PWM stays in **safe-off until Stage C1** (MOE/AutomaticOutput disabled).
- First power-on uses a **current-limited bench supply**, motor leads clamped/limited.
- Bring-up current/voltage limits start low; raise only after the loop is verified.
- Enable BKIN/break input and the independent watchdog as soon as PWM can move.
- If a stage needs a hardware value we don't have (Vbus divider, dead-time, phase signs),
  **stop and ask** — never invent (process rule #10).

## Watch/inject harness (`include/mc_debug.h`, built in A1)

- `volatile MC_Debug_t g_mc_debug` — observe framework internals (loop cadence, timing,
  power-stage state). Each module adds fields here as it lands.
- `volatile MC_Inject_t g_mc_inject` — command/inject from the watch window; **gated by
  `inject_enable`** so it can't drive hardware by accident. This is the command path until
  the OD/mode-manager exists. (Echoes the old `sensors_debug` / `comm_vars` / DAC-`iq`.)

## CubeIDE integration (one-time)

The framework lives in repo-root `include/` and `src/` (kept out of CubeMX `Core/`). Add
them to the build once: Project ▸ Properties ▸ C/C++ General ▸ Paths and Symbols ▸ add
`include` to Includes (GNU C), and add `src` as a Source Location. Rebuild.

---

## Phase A — Foundations & observability (no power)

- **A1 — Scheduler + debug harness** (`mc_scheduler`, `mc_debug`; wire TIM1/TIM7 ISRs).
  PWM safe-off. *Watch:* `g_mc_debug` counters + period/duration cycles.
  *Pass:* fast 20 kHz (period ≈ 8500 cyc), medium 1 kHz (≈ 170000 cyc), slow 100 Hz;
  counts ratio 20:1 (fast:medium) and 10:1 (medium:slow); `pwm_enabled=false`; no overruns.
- **A2 — PID + math** (`mc_pid`, `mc_math` + CORDIC wrapper). *Watch/inject:* step into a
  scratch PID via `g_mc_inject.scratch_f`. *Pass:* P/PI/PID, clamp, reset, deriv-filter
  correct; CORDIC sin/cos matches libm (golden vs old `foc.c`).

## Phase B — Sensing (no power)

- **B1 — Board config + current sense** (`mc_board_config` profile 0,
  `mc_current_sense_stm32g474`, offset cal; move fast-loop trigger to **ADC EOC**).
  *Pass:* zero-current ≈ 2124 counts; ≈ 0 A at rest; injected bench current reads correct
  sign & scale (≈ 0.01555 A/count). Golden vs old.
- **B2 — SSI encoder + state estimator** (`mc_ssi_encoder`, `mc_state_estimator`; observer +
  finite-diff selectable). Turn shaft by hand. *Pass:* 2,097,152 cnt/rev; angle tracks
  rotation; direction sign right; elec angle wraps pole_pairs× per mech rev; observer
  low-lag. Golden vs old.

## Phase C — Power stage & open loop (first PWM — careful)

- **C1 — PWM driver + safe-off** (`mc_pwm_stm32g474`). *Inject* fixed low duty; scope phase
  outputs + dead-time; test `ForceSafeOff`. *Pass:* complementary + dead-time correct;
  safe-off truly disables; over-limit duty rejected. **Needs dead-time confirmed.**
- **C2 — Open-loop voltage + rotor alignment** (small d-axis voltage; port old `alignment.c`).
  *Pass:* rotor locks to commanded angle; electrical offset captured; current within limit;
  matches old alignment.

## Phase D — Closed-loop control

- **D1 — FOC current loop** (`mc_foc`: Clarke/Park, d/q PI, voltage limit + anti-windup,
  inv-Park, SVPWM; `mc_current_request` torque→iq). Low iq limit. *Inject* iq_cmd (id=0).
  *Pass:* iq tracks step, stable; id≈0; anti-windup sane; transforms match old.
- **D2 — Velocity loop** (`mc_velocity_controller`, PI; observer velocity). *Inject*
  velocity_cmd. *Pass:* tracks steps, no sustained oscillation; observer≈finite-diff.
- **D3 — Position loop + trajectory** (`mc_position_controller` P; `mc_trajectory` S-curve,
  time-stretch, replan). *Inject* target + requested_time. *Pass:* smooth jerk-limited move,
  zero end-vel; stretch flagged when infeasible; clean replan. Golden vs old S-curve.

## Phase E — Supervisory

- **E1 — Mode manager + conditioners** (`mc_mode_manager`, `mc_command_conditioner`).
  *Pass:* modes route correctly; joystick bypasses trajectory/position; controlled stops;
  hold capture; fault latch.
- **E2 — Fault manager** (`mc_faults` severity matrix). *Pass:* correct warn/stop/safe-off
  action, debounce, latch+reset; severe faults force PWM off in the fast loop.
- **E3 — Calibration manager** (`mc_calibration`). *Pass:* routines run gated, abort/time-out
  safely, store results.
- **E4 — Persistence** (`mc_persistent_store`, versioned+CRC32). *Pass:* save→reboot→load
  round-trips; bad CRC → defaults + warning; no flash writes in fast/medium loops.

## Phase F — Comms (OD + inter-MCU, last)

- **F1 — Object dictionary** (`mc_od` + object map). *Pass:* typed read/write with
  range/access enforcement; side-effects (shadow gains, save latch) correct.
- **F2 — Inter-MCU SPI link** (`mc_spi_protocol` integration + the 4 non-cyclic payloads +
  SPI2-slave DMA, heartbeat/timeout). *Pass:* framed exchange with CRC/version/seq checks;
  cyclic command drives motion; OD read/write over the link; SPI-timeout → quick-stop.

Then a short **integration + tuning** pass and locking the public APIs.
