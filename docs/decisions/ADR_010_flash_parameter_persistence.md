# ADR-010: Flash parameter persistence architecture

## Status

Accepted — architecture; first implementation is **calibration-only** (see Resolution).

## Date

2026-06-19

## Context

Runtime configuration and calibration must survive power cycles so the generic binary can run
a given motor/board without recompiling and so calibration (the C2 electrical offset, current
offsets, encoder zero, etc.) isn't lost. STM32G474RE: 512 KB flash, 2 KB pages, double-word
(64-bit) programming, erase-by-page. No flash writes are allowed in the fast/medium loops.

## Decision

Two layers:

1. **Generic NV store** (`mc_persistent_store`): owns a reserved flash region at the top of
   flash (carved out in the linker script so code never uses it). It reads/writes one record:
   `header { magic 'MCPF', version, size_bytes, crc32 }` + `payload`. On load it validates
   magic, version, size, and CRC32. It uses an **A/B two-page ping-pong** with a sequence
   counter, so a power loss mid-write cannot corrupt the last-good copy (single-page is the
   minimal fallback). Saves are **latched and executed only in the slow/background context**
   (HAL_FLASH unlock → page erase → program double-words → lock).

2. **Schema layer** (`mc_params`): defines `MC_Params_t`, the payload — the persistable
   configuration, embedding the **pointer-free** module config structs plus calibration
   results. It never stores runtime pointers/handles. It provides `LoadDefaults` (from the
   compile-time profile loaders), `CaptureFromLive` (snapshot live → params before a save), and
   `ApplyToLive` (params → live module configs after a load).

**Boot flow:** `Init` reads NV → if valid, `g_params = stored`; else `g_params = defaults` and a
"params defaulted" warning is raised. `ApplyToLive(&g_params)` pushes config into the live
modules before the loops run.
**Save flow:** `RequestSave` latches a flag; `ServiceSlow` (slow loop) does
`CaptureFromLive(&g_params)`, computes CRC, erases the inactive page, programs it, marks it active.
**Factory reset:** invalidate NV → next boot loads defaults.
**Versioning:** bad CRC or version mismatch → load defaults (no migration initially, per spec 14).

## Reasoning

Mirrors the proven `flash_params` approach (flat versioned + CRC32 blob, compile-time defaults)
but split into a generic store + a schema layer to suit the generic framework. A/B pages add
power-loss safety cheaply. Excluding pointers keeps the blob valid across boots. Confining flash
writes to the slow context honours the real-time rules.

## Consequences

- New `include/mc_params.h` (schema) + later `src/mc_params.c` and the STM32 flash driver
  (`src/mc_persistent_store_stm32g474.c`).
- The linker script reserves the NV region (e.g. the top 1-2 pages of flash).
- The live module configs must be reachable by `CaptureFromLive`/`ApplyToLive` — they are
  currently statics in `mc_scheduler`, so they should move to a small **axis/config registry**
  (or the scheduler exposes accessors). This is the main refactor this introduces.
- The C2 electrical offset and the current-sense offsets become persistable.

## Files affected

- include/mc_params.h, include/mc_persistent_store.h
- docs/spec/14_persistence.md
- (later) src/mc_params.c, src/mc_persistent_store_stm32g474.c, STM32G474RETX_FLASH.ld

## Resolution (first implementation, 2026-06-21)

User decisions: **calibration-only** payload, **auto-save on alignment capture**, **A/B
two-page**. Implemented:

- **Generic store** `mc_persistent_store` (HAL-free): `MC_ParamStoreHeader_t` (magic 'MCPF',
  version, payload_size, **seq**, crc32) + payload; A/B ping-pong; newest valid `seq` wins;
  verify-after-write before switching the active slot. CRC32 reflected poly 0xEDB88320 (matches
  the proven `flash_storage`). Save latched, written in the slow/background context.
- **Flash port** `mc_flash_port` + `mc_flash_port_stm32g474.c` (boundary): two slots = bank 2
  pages 126/127 at **0x0807F000 / 0x0807F800** (top of flash, opposite the bank-1 code so
  read-while-write is possible). Dual-bank, 2 KB pages — **confirmed** from the old code, not
  assumed. Double-word programming.
- **Payload** `MC_CalibData_t` (24 B): electrical offset, current offsets A/C, mechanical zero,
  phase order. Embedded in `MC_Params_t` so the full-params schema stays consistent.
- **Linker**: FLASH length 512K → 508K to reserve the top 4 KB.
- **Flow**: boot loads + applies the record (electrical offset → estimator, current offsets →
  current-sense); `request_align_capture` captures the offset AND latches a save; the slow loop
  writes it **only while the power stage is off** (`s_pwm_on == false`). `request_factory_reset`
  erases both slots. Watch mirrors: `store_valid`, `store_save_pending`.

## Open questions

- Centralise live configs into an axis/config registry (needed for the **full** MC_Params_t).
- Extend to the full params blob (gains/limits/motor/board) — version bump, reloads defaults once.
- Relax the "drive off" save gate using bank-2 read-while-write, if saving during drive is wanted.
- Whether current-offset calibration should also auto-save (today only alignment capture does).
