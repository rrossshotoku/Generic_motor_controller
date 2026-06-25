# ADR-033: Cyclic header v4 — position_actual + movement_status (REQ-0013)

- **Status:** Accepted
- **Date:** 2026-06-24
- **Related:** REQ-0013 (CMC→motor), ADR-016 (inter-MCU SPI protocol), ADR-028 (D3 position cascade)

## Context

REQ-0013 (raised by the CMC): the CMC's CAMERAD shot-store captures the axis position the instant the
operator presses STORE, and needs a live moving / on-target indicator for the panel. Today position lives
only in the **host-configurable** telemetry blob (`0x6064` mapped via `0x2A00`), so the CMC can't rely on
it — any GUI remap breaks the shot system — and the moving/on-target bits are faked. The fix is to put live
position and a movement-status bitfield in the **fixed** cyclic status header so they're always present,
independent of the telemetry map.

## Decision

Extend `MC_IfCyclicStatusHeader_t` (the fixed cyclic status header) with two fields:

```c
int32_t  position_actual_scaled;  /* OD 0x6064, MC_IF_POS_SCALE (1e-5 rad/LSB) */
uint16_t movement_status;         /* MC_IF_MOVE_* bits */
```

This is the **first wire-breaking change**: `MC_IF_STATUS_HEADER_SIZE` 12 → 18, telemetry blob budget
`MC_IF_TLM_BLOB_MAX` 40 → 34, and **`MC_IF_PROTOCOL_VERSION` 3 → 4**. A version mismatch is rejected
(`MC_IF_ERR_BAD_VERSION`), so motor + CMC + GUI must be updated/flashed together.

**`movement_status` bits — agreed reduced set.** REQ-0013 proposed six bits; trimmed to the ones actually
consumed (user decision, 2026-06-24):

| Bit | Name | Meaning | Populated |
|----|------|---------|-----------|
| `0x0001` | `MC_IF_MOVE_MOVING` | axis in motion: drive enabled **and** ( \|velocity demand\| or \|measured velocity\| > 0.01 rad/s ) | live |
| `0x0002` | `MC_IF_MOVE_ON_TARGET` | position-loop target reached (mirrors the D3 `target_reached` / `SW_TARGET_REACHED`) | live |
| `0x0010` | `MC_IF_MOVE_AT_LIMIT_LO` | soft min-position limit hit | **0 — reserved** |
| `0x0020` | `MC_IF_MOVE_AT_LIMIT_HI` | soft max-position limit hit | **0 — reserved** |

`0x0004` / `0x0008` (REQ-0013's `SETPOINT_ACCEPTED` / `SETPOINT_COMPLETE`) are **dropped → reserved**; the
CMC's stated consumers use only `MOVING` and `ON_TARGET`. Bit positions are kept from REQ-0013 so the CMC's
references still match and nothing renumbers if the dropped/reserved bits are ever added. `AT_LIMIT` bits
are reserved (always 0) because the motor has **no soft position limits yet** — adding them is a fast
follow (separate ADR) and needs no further wire change.

**Seam.** The scheduler computes `movement_status` each medium tick (in `od_mirror_live`, from
`target_reached` + the velocity demand/measured) and **pushes** it to the comms layer via
`MC_Comms_SetMovementStatus()` — the existing scheduler→comms direction (no circular dependency).
`build_telemetry` reads `position_actual` from `0x6064` through the OD (exactly as it already reads
`0x6041`/`0x603F`/`0x6061`) and stamps `movement_status` from the pushed value. `movement_status` stays a
**header-only** field (not a new OD object), matching REQ-0013's intent that it live in the fixed header.

## Consequences

- First-ever `MC_IF_PROTOCOL_VERSION` bump (3 → 4); **coordinated flash** of motor + CMC + GUI.
- The CMC (separate project) updates its `axis_manager` consumers per REQ-0013 — not in this repo.
- GUI cyclic-header parser: 12 → 18 bytes, +2 fields; telemetry blob budget now 34; `movement_status`
  available for the status display.
- The reduced scope (dropped setpoint bits, reserved AT_LIMIT) is recorded as a note on REQ-0013 in
  `REQUESTS.md` so the CMC author re-syncs.

## Files

`../Lightweight_CMC/Interface/mc_if_protocol.h` (shared), `CHANGELOG.md` [4.0.0], `REQUESTS.md`
(REQ-0013 note); `src/mc_comms.c` + `include/mc_comms.h`, `src/mc_scheduler.c`, `include/mc_debug.h`
(watch-window mirror); `gui/mc_gui/` cyclic-header parser + `smoke_test.py`.
