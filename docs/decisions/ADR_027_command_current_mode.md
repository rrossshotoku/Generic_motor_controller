# ADR-027: PC command surface + current/torque mode via the CMC axis_manager

## Status

Accepted

## Date

2026-06-22

## Context

The PC tool is getting a **motor command page** (set mode; request velocity / position / current). Per
`INTERFACE_SPEC.md` §5b the PC tool MUST command through the CMC `axis_manager` (`0x30xx`), not by
writing the motor's CiA-402 objects (`0x6040`/`0x6060`/`0x60FF`/`0x6071`) directly — those stay
visible for debug/tuning only, and bypassing `axis_manager` would desync `axis_state` and break
arbitration between protocol modules.

State of play (trust-the-code):
- **Velocity**: works end-to-end — `axis_manager` streams `velocity_setpoint` in the cyclic frame; the
  motor's remote path routes `PROFILE_VELOCITY`.
- **Current/torque**: the **motor side is already wired** — `mc_scheduler.c` routes
  `MC_MODE_TORQUE_CURRENT` (`0x6060 = MC_IF_MODE_TORQUE = 4`) to `s_eff_iq_cmd = target_torque (0x6071)
  × MC_IF_CUR_SCALE` (i.e. amps). But the **CMC `axis_manager` has no torque mode and no target-current
  entry** (`compute_desired` hardcodes `target_torque_scaled = 0`), so current is not reachable through
  the mandated path.

User direction (asked): wire current end-to-end through `axis_manager`, commanded in **amps**.

## Decision

Add a current/torque command mode to the CMC command surface; **the motor side is unchanged**.

**Contract (`mc_if_od.h`):**
- New CMC-owned OD entry **`0x302B axis_target_current`** (`F32`, `RW`, `OWNER_CMC`) — the commanded
  current in **amps** (maps to the motor's `0x6071 target_torque`, scaled `MC_IF_CUR_SCALE = 1e-3 A/LSB`).
- New axis op-mode **`MC_IF_AXIS_MODE_TORQUE (5u)`** (the existing values stop at `HOLD = 4`).

**CMC `axis_manager`:**
- `AXIS_OP_MODE_TORQUE`; `axis_mode_to_cia402(TORQUE) → MC_IF_MODE_TORQUE (4)`.
- `compute_desired` sets `target_torque_scaled` from `axis_target_current` (A → `0x6071` via
  `MC_IF_CUR_SCALE`); the existing `SEQ_TARGET_TORQUE` sequencer step SDO-writes `0x6071`.
- Getter/setter + `cmc_od` dispatch for `0x302B`.

**Set-and-hold via SDO**, not streamed: the v3 cyclic frame carries only `controlword` +
`velocity_setpoint`; adding torque to the cyclic would be a wire-layout change (protocol bump). Current
is therefore a setpoint you set and hold (SDO-rate), not a high-rate live jog — adequate for a GUI/test
surface; live current jogging would need a future cyclic-frame revision.

## Units

**Amps.** `0x302B` (CMC) and `0x6071` (motor) are both currents in A; the motor converts to torque via
`Kt` internally (`iq → torque = iq·Kt`). "Request a current" = command `iq` directly.

## Verification

- `gcc -fsyntax-only` on the touched motor + CMC C.
- GUI smoke test parses `0x302B`, the command page issues mode/target writes.
- On-target (user): mode = Torque, enable, set a small current, confirm `torque_actual (0x6077)` and
  motion; quick-stop / disable cuts it.

## Consequences

- Current is commandable through the spec-mandated path; the command page gets a Torque mode.
- No motor-firmware or wire change; additive contract only.
- High-rate current jogging is out of scope (SDO set-and-hold) until/unless the cyclic frame is revised.

## Contract impact (governance)

Additive CMC-owned RW entry + a new axis-mode constant in `mc_if_od.h`; **no wire/PDO-layout change →
`MC_IF_PROTOCOL_VERSION` stays 3**. The CMC owns these `0x30xx` entries, so — per the user's direction —
the contract additions and the `axis_manager`/`cmc_od` implementation are **delegated to the CMC via
`Interface/REQUESTS.md` REQ-0012** and are *not* pre-applied here (no `CHANGELOG` entry until the CMC
implements; the CMC adds it then). Consumers: **CMC** adds the entries + implements; **motor MCU**
unchanged; **PC tool** adds the Torque control, shown as pending REQ-0012 until the CMC lands it.

## Files affected

- `../Lightweight_CMC/Interface/REQUESTS.md` — **REQ-0012** (full spec of the CMC work: the contract
  additions + `axis_manager` + `cmc_od`). The contract/CMC changes are the CMC's to make.
- `../Lightweight_CMC/Interface/gui/mc_gui/main_window.py` (PC command page; Torque shown pending REQ-0012)

## Open questions

- Live (cyclic-rate) current/torque streaming would need a cyclic-frame revision (protocol bump) — deferred.
