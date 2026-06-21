# ADR-017: CiA-402 OD entries, extended result codes, ERROR staging, dead-code removal

## Status

Accepted

## Date

2026-06-21

## Context

The network MCU (`Lightweight_CMC`) audited the motor MCU against the shared contract and filed
`Interface/REQUESTS.md` REQ-0001..0006 — concrete gaps blocking end-to-end OD-over-SPI/UDP. Its
`app/cia402` is a Phase-5 stub (returns `NOT_READY`) and its `app/od` UDP bridge matches
`NETWORK_UDP_SPEC.md`, so the work is on the motor side: implement the requested entries/codes.

## Decision

Implement REQ-0001, 0002, 0003, 0005, 0006; defer REQ-0004.

- **REQ-0001 (CiA-402 entries):** added the 17 standard `0x1xxx`/`0x6xxx` objects to `mc_od.c`,
  bound to `g_od` backing fields. RO actuals (`0x6064`/`0x606C`/`0x6077`, statusword, error) are
  mirrored from live state, **scaled to wire units** (`MC_IF_*_SCALE`) each medium loop; RW
  objects (controlword/modes/targets/profile params) are stored. OD read/write now succeeds for
  all indices (unblocks integration). The controlword→CiA-402 **state machine** behaviour is the
  **mode manager's** job (E1), which consumes these stored RW objects — not in this ADR.
- **REQ-0002 (result codes):** `MC_OdStatus_t` gains `NO_SUB` + `NOT_READY`; `MC_Od_*` return
  `NO_SUB` when an index exists but the subindex doesn't (`od_notfound`); `mc_comms` maps all 9.
- **REQ-0003 (6 manufacturer entries):** `0x2000:3/4` (R/L), `0x2600:1` fault_flags, `0x2700:2`
  cal_status, `0x2800:2` store_status, `0x2800:3` factory-reset (magic → slow-loop reset). Also
  wired `0x2800:1` save magic → `calib_save`.
- **REQ-0005 (ERROR staging):** `mc_comms` emits `MC_IF_MSG_ERROR` (class/detail/ref_sequence)
  as the next transaction's frame on frame-validation failure or unknown message type.
- **REQ-0006 (dead code):** deleted `include/mc_spi_protocol.h`; removed the stale
  `MC_SPI_PROTOCOL_VERSION`/`MC_SPI_MAX_PAYLOAD` from `mc_config.h`.
- **REQ-0004 (0x2A00 into OD table): deferred** — wire behaviour already works; needs OD-engine
  array support + index/subindex context in the write callback; folded into the unified
  config-registry cleanup.

## Reasoning

The blocking requests (entries exist + result codes) unblock the CMC's OD bridge with bounded,
additive changes, consistent with the existing g_od mirror/apply pattern (RO mirrored, RW
stored). The deep CiA-402 state-machine behaviour is correctly the mode manager's responsibility,
keeping the dependency direction OD → controller. REQ-0004 is deferred to avoid a half-done
engine change.

## Consequences

- `mc_od.c` table grows to the CiA-402 + manufacturer set; `g_od` gains the backing fields; the
  scheduler mirrors the scaled actuals + status and services the OD persistence magics.
- `mc_comms` reports protocol errors; the motor build depends on `mc_if_od.h` (scale constants).
- A PC tool can now read all CiA-402 indices over UDP→SPI and get live values; writing
  controlword does **not** yet drive the state machine (E1).
- The mirror duplicates data across `g_mc_debug`/`g_od`/live configs — the config-registry
  cleanup (with REQ-0004) remains the consolidation step.

## Files affected

- include/mc_od.h, src/mc_od.c, include/mc_od_store.h
- src/mc_comms.c, src/mc_scheduler.c, include/mc_config.h, (deleted) include/mc_spi_protocol.h
- docs/spec/05_object_dictionary.md
- ../Lightweight_CMC/Interface/REQUESTS.md (REQ-0001/2/3/5/6 → done; 0004 deferred)

## Open questions

- Mode manager (E1): CiA-402 state machine consuming controlword/modes/targets.
- REQ-0004 + config registry; CiA-402 RW objects (target_velocity etc.) currently stored-only.
