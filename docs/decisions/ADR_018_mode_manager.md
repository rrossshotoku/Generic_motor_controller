# ADR-018: Mode manager (E1) + commissioning-vs-remote command arbitration

## Status

Accepted

## Date

2026-06-21

## Context

The CiA-402 command path (controlword → enable, modes_of_operation → routing) had no
implementation: the OD RW objects were stored but nothing acted on them (REQ-0001 caveat). The
cyclic-command apply in `mc_comms` poked the bring-up harness (`g_mc_inject`/`g_mc_debug`)
directly — a reuse coupling. E1 implements the mode manager and uses it as the seam to both make
the CiA-402 path real and decouple `mc_comms`. The motor is under active SWD bring-up, so the
watch-window command path must be preserved.

## Decision

- **`mc_mode_manager.c`**: a simplified CiA-402 state machine driven by the **contract's**
  controlword bits (CW_ENABLE / CW_QUICK_STOP / CW_FAULT_RESET / CW_HALT — single-bit, not the
  full multi-bit encoding) → states Disabled / Ready / Enabled / Quick-Stop / Fault, producing
  the `statusword` and the active `MC_Mode_t`. Severe fault latches until a fault-reset edge.
  Fault input is minimal (over-current trip → severe) until the fault manager (E2).
- **Command arbitration (commissioning vs remote)** in the scheduler medium loop:
  - `inject_enable == true` → **commissioning**: the watch-window path drives, *exactly* as
    before (foc_enable / velocity_enable / iq/velocity cmd / open-loop align). Unchanged bench flow.
  - `inject_enable == false` → **remote**: the mode manager (reading OD `0x6040/0x6060/0x60FF/
    0x6071`) drives. Boot-safe: controlword defaults 0 → Disabled → safe-off.
  The arbiter computes one **effective command** (`s_eff_*`: drive enable, torque-vs-velocity
  mode, iq/id/velocity setpoint, open-loop align) that the fast/medium loops consume — replacing
  the scattered `g_mc_inject` gating. Routed modes: Disabled, Profile/Joystick Velocity, Torque
  (the loops that exist); Position/Homing recognised but not routed until D3/calibration.
- **`mc_comms` decoupled**: the cyclic command now **writes the OD** (controlword/mode/targets
  via `MC_Od_Write`); telemetry reads the statusword/error from the OD (`MC_Od_ReadRaw`). The
  `mc_debug.h` dependency (`g_mc_inject`/`g_mc_debug`) is removed — `mc_comms` depends only on the
  OD + shared headers, so it is reusable.
- The statusword (OD `0x6041`) is owned by the arbiter: derived from the live drive in
  commissioning, from the mode manager in remote.

## Reasoning

Following the contract's simplified bits keeps the motor MCU and a CiA-402 host in agreement
without the full state-machine ceremony. The commissioning/remote split makes the CiA-402 path
live while preserving the bench workflow exactly and keeping power-up safe-off. Routing commands
through the OD removes the comms↔harness coupling (the reuse goal) without a full config-registry
rewrite.

## Consequences

- New `mc_mode_manager.c`; scheduler gains the arbiter + `s_eff_*` effective command; the
  fast/medium loops consume it instead of raw `g_mc_inject` flags (commissioning branch preserves
  the old behaviour bit-for-bit).
- `mc_comms` no longer includes `mc_debug.h`; writes/reads the OD. (Removes coupling #1 from the
  reuse audit.)
- Writing OD `0x6040 = CW_ENABLE` with `inject_enable == false` now energises the drive in the
  selected mode — closes the REQ-0001 caveat for velocity/torque (position needs D3).
- **On-target: re-verify the commissioning bench flow** (align / jog / tune) after flashing — the
  command path was restructured though commissioning behaviour is intended to be identical.

## Files affected

- src/mc_mode_manager.c, include/mc_mode_manager.h
- src/mc_scheduler.c (arbiter + effective command; fast/medium loops)
- src/mc_comms.c (OD-driven; mc_debug.h removed)
- docs/spec/07_mode_manager.md

## Open questions

- Full fault manager (E2) feeding `MC_FaultState_t` (severe/recoverable/warning + quick-stop).
- Position/homing routing (needs D3 position loop + trajectory).
- Quick-stop controlled deceleration profile (currently → safe-off).
