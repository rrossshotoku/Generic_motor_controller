# ADR-021: Adopt v3 cyclic command (streaming velocity_setpoint; joystick stays CMC-side)

## Status

Accepted

## Date

2026-06-22

## Context

Interface **v3** (`CHANGELOG.md` [3.0.0]) reshapes `MC_IfCyclicCommand_t` from the v2 8-field /
31-byte form to **10 bytes**: `{controlword, velocity_setpoint (i32 ×MC_IF_VEL_SCALE),
command_counter}`. Everything else — `mode_of_operation`, all targets, profile params,
`target_position_time_ms` — is now **SDO-only**: the host writes setup via the OD pipeline, then
triggers execution by rising-edging the new `MC_IF_CW_NEW_SETPOINT` (`0x0010`). New OD entry
`0x607B target_position_time_ms`. `MC_IF_PROTOCOL_VERSION` 2 → 3.

The original v3 draft streamed a normalised `joystick_value` + a motor-side `joystick_scale_rad_s`
OD entry. The motor side raised **REQ-0010** (accepted, re-cut in place): stream a **velocity**
directly and keep joystick→velocity scaling in the CMC's `axis_manager` — a *generic* motor
controller has no joystick concept, and the draft round-tripped velocity → i16 → velocity.

## Decision (motor side)

- **`apply_cyclic` (mc_comms)** unpacks the 10-byte command: `controlword`→`0x6040`,
  `velocity_setpoint`→`0x60FF`, `command_counter`→dead-man. It no longer writes
  `mode_of_operation`/`target_position`/`target_torque`: those are **SDO-owned** now and persist in
  the OD. This is the key behavioural win — the cyclic stream no longer clobbers them, so a host
  SDO write to `0x6060`/`0x607A`/`0x6071` finally **sticks**.
- **`velocity_setpoint` is the authoritative live demand.** It lands in `0x60FF`
  (`g_od.target_velocity`), which the scheduler's `PROFILE_VELOCITY` path already consumes — so the
  velocity loop runs from the cyclic value. A host SDO write to `0x60FF` is informational
  (overwritten each frame), exactly as the contract specifies. The slow-loop dead-man still zeroes
  `g_od.target_velocity` on a stale stream (stops the motor).
- **JOYSTICK removed.** Dropped `MC_MODE_JOYSTICK_VELOCITY` (the `MC_Mode_t` enum value, the
  mode-manager `switch` case, and the scheduler's velocity-routing branch). Velocity is
  `PROFILE_VELOCITY` only; the CMC's JOYSTICK op-mode maps to motor `PROFILE_VELOCITY` + a streamed
  velocity.
- **NEW_SETPOINT seam.** The scheduler decodes the controlword bit into `dc.new_setpoint`; the mode
  manager detects its **rising edge** into `ds.new_setpoint_latched` (one-shot). This is the
  trigger for a `PROFILE_POSITION` move — scaffolded now, **execution deferred to D3** (position is
  still recognised-but-inert without the position loop + trajectory engine).
- **`0x607B target_position_time_ms`** added as a `g_od` field → auto-generated into the OD table
  (ADR-020). Default 0 (= ASAP).
- **Version 3** is accepted automatically via the shared `MC_IF_PROTOCOL_VERSION` constant on
  rebuild (validate + stamp); v2 frames are rejected with `MC_IF_ERR_BAD_VERSION`.

## Consequences

- **Host velocity control works end-to-end now:** SDO-write `0x6060 = 3` (sticks), enable via the
  cyclic controlword, stream `velocity_setpoint` → the motor runs the velocity loop. This resolves
  the v2-era problem where the quiescent cyclic stream stamped `0x6060`/`0x60FF` back to zero every
  millisecond.
- `PROFILE_POSITION`-on-NEW_SETPOINT is deferred to D3; the edge-detection seam is in place
  (`ds.new_setpoint_latched`).
- **Stale joystick references to clean up when next touched** (out of scope here, flagged so they
  aren't forgotten): the *unbuilt* `include/mc_command_conditioner.h` still has a
  `joystick_normalised` parameter, and `docs/spec/07_mode_manager.md`'s routing table still lists a
  "Joystick Velocity" mode. Per REQ-0010 the motor has no joystick concept — the conditioner should
  take a normalised/velocity input, and the spec mode list drops Joystick Velocity.

## Verification

- `gcc -fsyntax-only -std=c11 -I include -I ../Lightweight_CMC/Interface src/mc_comms.c src/mc_od.c
  src/mc_mode_manager.c` → clean against the v3 headers.
- OD table dump: **63 entries** (62 + `0x607B`); `0x607B` present; no joystick entry.
- **On-target (pending):** rebuild v3; confirm SDO `0x6060=3` + controlword enable + streamed
  `velocity_setpoint` runs the velocity loop, `velocity_setpoint=0` stops it; a v2 frame is rejected
  with `MC_IF_ERR_BAD_VERSION`.

## Files affected

- src/mc_comms.c (apply_cyclic), include/mc_od_store.h (`target_position_time_ms`),
  src/mc_mode_manager.{c,h} (drop JOYSTICK; NEW_SETPOINT edge), src/mc_scheduler.c (decode
  NEW_SETPOINT; drop JOYSTICK branch)
- docs/decisions/ADR_021_v3_cyclic_command.md, docs/decisions/ADR_000_decision_log.md,
  docs/spec/05_object_dictionary.md, docs/spec/06_spi_protocol.md, docs/spec/07_mode_manager.md
- ../Lightweight_CMC/Interface/REQUESTS.md (REQ-0009 / REQ-0010 motor-side resolution)

## Open questions

- D3 (position loop + trajectory engine) to consume `new_setpoint_latched` and execute
  PROFILE_POSITION moves with `0x607B` timing.
- Joystick cleanup in `mc_command_conditioner.h` + spec 07 when those are next built/edited.
