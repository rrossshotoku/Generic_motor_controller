# Generic Motor Controller — Project Guide

Re-architecture of a proven single-axis BLDC/PMSM FOC controller
(`../bldc_axis_controller`) into a **generic, motor-type-agnostic framework** on the
STM32G474RET3. Internal units are **SI** (rad, rad/s, Nm, A, V, degC). A future
brushed-DC/H-bridge backend must remain possible, so outer motion layers never
depend on FOC concepts.

## Source of truth & MANDATORY documentation process

Plain-text project files are the source of truth — **never** the Word/PDF under
`Documentation/`. Read `docs/CODING_AGENT_DOCUMENTATION_UPDATE_PROCESS.md` and follow it.

**Core rule:** whenever a design decision is made, changed, or discovered, record it
in `docs/decisions/` (an ADR) **before or alongside** the code — not only in code or chat.

Priority order (if these disagree, STOP and flag):
1. `docs/decisions/*.md` (ADRs) — why decisions were made
2. `requirements.yaml` — machine-readable agreed requirements
3. `docs/spec/*.md` — subsystem specifications
4. `include/*.h` — C interface contracts (Doxygen comments)
5. `src/*.c` — implementations

**Update sequence for any architectural change:** ADR file → `ADR_000_decision_log.md`
→ `requirements.yaml` → `docs/spec/*.md` → `include/*.h` → `src/*.c` → `diagrams/*.dot`
→ tests → session summary in the response. Create a *separate* ADR when a decision
touches architecture, public C interfaces, the OD, SPI protocol, timing domains,
control loops, faults, calibration, persistence, or a motor/encoder backend.

End every working session with a short summary (Implemented / Decisions / Files changed /
Tests run / Open questions).

## Working principles

- **Hold the final architecture in mind even when building piecewise.** Every incremental
  bring-up step (the `mc_debug` watch/inject harness, per-stage shortcuts, RAM-only values)
  must fit the end-state design in `docs/decisions/` + `docs/spec/`. Prefer the
  architecturally correct seam over a local hack; when a bring-up shortcut is unavoidable,
  note it and the path back to the target design in the relevant ADR/spec.
- **Ask, don't assume.** When a change has a forking decision — storage layout, public
  interfaces, scope, hardware values, anything hard to reverse — ask clarifying questions
  before implementing instead of guessing. This is a hard requirement, not a preference.

## Canonical layout

```
include/            mc_*.h public contracts (host-compilable; no HAL types)
src/                mc_*.c implementations (clean-room; populated per task sequence)
tests/              host-buildable unit + golden-reference tests
docs/spec/          00..16 subsystem specs + 17_board_and_motor_config.md
docs/decisions/     ADRs + ADR_000_decision_log.md
diagrams/           Graphviz *.dot
requirements.yaml   agreed requirements
agent_tasks/        00_task_sequence.md (implementation order)
Core/, Drivers/     CubeMX-generated; DO NOT edit outside USER CODE sections
Documentation/      original planning bundles (historical origin) + Word source
```
The repo-root tree above is canonical. `Documentation/mc_coding_agent_pack` and
`Documentation/coding_agent_doc_process` are the historical bundles it was assembled from.

## Locked decisions (see ADRs)

- **ADR-001** Scope: build the **motor-control MCU only**; the inter-MCU SPI link is a
  frozen boundary contract. Network MCU + external protocol = separate project.
- **ADR-002** Migration: **clean-room from the specs**, each module validated numerically
  against `../bldc_axis_controller` as a golden reference.
- **ADR-003** Velocity feedback: **position-tracking observer** (Ellis, encoder-tracking
  PI + velocity damping) as default; finite-difference+LPF as runtime-selectable fallback.
- **ADR-004** Hardware: **per-board config** (`MC_BoardConfig_t`) + generic
  `MC_MotorModel_t`; ship board **profile 0** = the proven reference board.

## Hard rules

- C, not C++. `float` for control math unless a spec says fixed-point/CORDIC.
- All framework code in `mc_*` modules. Keep FOC concepts (id/iq, Park/Clarke, SVPWM,
  electrical angle) inside the BLDC/FOC backend. Public motor command = torque/current
  request, not `iq`.
- Controllers must not read the OD directly; convert scaled-int ↔ SI float at the OD boundary.
- No dynamic allocation, blocking, SPI, or flash writes in the 20 kHz fast loop; no flash
  writes in the medium loop. Persistence runs only in the slow/supervisory context.
- Encoders (SSI now, quadrature later) stay behind the common position-feedback interface.
- HAL/CubeMX owns peripheral setup; LL/direct-register access only in the STM32 boundary
  modules (`mc_*_stm32g474.c`).
- Hardware-specific values live in board profiles, not `#define`s. If implementation needs
  a hardware value not in the spec, STOP and report it.

## Timing domains

Fast 20 kHz (TIM1 update ISR): current read, angle, FOC, PWM. Medium 1 kHz (TIM7): SSI +
state estimator, trajectory, position/velocity loops, current request. Slow 100 Hz
(decimated from TIM7 ÷10): OD/SPI service, faults, thermal/derating, persistence.

## Build & test

- **Primary: incremental on-target bring-up over SWD** (ADR-005). Build in STM32CubeIDE
  (repo-root `.cproject`), flash, and observe/drive behaviour in the debugger **live watch
  window**. Each module exposes a `volatile` debug-mirror snapshot + guarded `volatile`
  command/inject fields (echoes old `sensors_debug` / `comm_vars` / DAC `iq`). Build
  debug-friendly (-Og/-O0, full symbols) so watched/injected vars aren't optimised away.
- Golden reference: compare live values/behaviour against `../bldc_axis_controller` (ADR-002).
- Host tests (Unity) are **optional**, not a gate. `mc_*` modules stay HAL-free and
  host-compilable regardless, so off-target checks remain possible later.
- Repo is **not yet under git** — `git init` before substantial implementation.

## Reference & hardware

Golden reference + proven constants: `../bldc_axis_controller` (`Core/Inc/motor_config.h`,
`Core/Src/foc.c`, `sensors.c`, `motion_*.c`). Board profile 0: current sense shunt 0.01 Ω,
gain 5.18, offset 1.71 V, 12-bit ADC @ 3.3 V; PWM 20 kHz, ARR 4250; SSI 21-bit (2,097,152
cnt/rev) on SPI1; inter-MCU SPI2 slave. **Confirm per board:** Vbus divider ratio, dead-time
counts, temperature-sense scaling, phase-current signs.
