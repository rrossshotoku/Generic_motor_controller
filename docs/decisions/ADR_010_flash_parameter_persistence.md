# ADR-010: Flash parameter persistence architecture

## Status

Accepted (architecture; implementation in Phase E)

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

## Open questions

- Exact NV region address/size and the linker reservation.
- A/B two-page vs single-page (recommend A/B for power-loss safety).
- Centralise the live configs into an axis/config registry now, or add accessors later?
- Final `MC_Params_t` field set (review the schema).
