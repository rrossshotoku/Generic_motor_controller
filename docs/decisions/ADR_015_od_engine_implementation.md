# ADR-015: Object dictionary engine + table (motor side, F1)

## Status

Accepted

## Date

2026-06-21

## Context

Implementing the motor-MCU side of the shared contract (ADR-013/014). First the OD — the typed
key/value store the SPI transport and the tuning GUI act on. The shared canonical map is
`MC_IF_OD_OBJECTS` in `../Lightweight_CMC/Interface/mc_if_od.h`.

## Decision

- **Engine** (`mc_od.c`, against the existing `mc_od.h`): static table; `Find` (linear);
  typed `Read`/`Write` with access (RO/WO/RW, bitwise), type, size, and range checks and optional
  read/write callbacks; typed accessors (U16/I32/Float).
- **Backing store** (`mc_od_store.h` / `g_od`): one struct holding the OD-exposed values; table
  entries bind their `data` pointer to its fields. Manufacturer objects are **float32 SI** (exact),
  matching the shared map.
- **Scope (F1):** a **tuning-focused subset** — gains (position/velocity/current/observer),
  limits, key telemetry (id/iq, vel demand/actual/cmd, vd/vq, angle, position, velocity, bus),
  motor params, and commissioning placeholders. The CiA-402 standard objects (0x6xxx) and the
  0x2A00 telemetry map land with the **SPI transport + mode manager** (they need scaling
  callbacks and the mode latch).
- **Integration (additive, low-risk):** `MC_Od_Init` seeds `g_od` (gains equal the configs
  already seeded). The **slow loop** applies OD-written gains (`od_apply_gains`) to the live
  controller configs at a safe point — so writing a gain via the OD tunes the loop. The
  **medium loop** mirrors live telemetry into `g_od` (`od_mirror_live`). Fast/medium control math
  is unchanged. Observer gains and the electrical offset stay on the watch-window / alignment
  paths for now (read-reflected in the OD; OD writes to them not yet applied) to avoid a
  two-writer conflict; they fold in with the unified config registry later.

## Reasoning

A correct, self-contained engine plus a small bound store gets live gain-tuning and telemetry
working immediately, on the existing harness, without a risky config-registry refactor. The
subset is exactly what the tuning GUI needs; the rest is sequenced with the transport that uses it.

## Consequences

- New `src/mc_od.c`, `include/mc_od_store.h`. Scheduler: seed + slow-loop apply + medium-loop
  mirror.
- OD gains are now the source applied to the controllers (via the slow loop) — gain writes take
  effect within ~10 ms.
- Table is a hand-maintained subset of `MC_IF_OD_OBJECTS`; keep in sync (full generation from the
  shared X-macro, and binding the CiA-402 objects, is a later step).
- Next: SPI-slave transport (decode/dispatch/stage responses, build the telemetry frame), the
  0x2A00 map + gather, and CiA-402 object scaling + mode-manager wiring.

## Files affected

- include/mc_od.h (existing), src/mc_od.c, include/mc_od_store.h
- src/mc_scheduler.c
- docs/spec/05_object_dictionary.md

## Open questions

- Unified config registry so OD is the single source for ALL configs (folds in observer gains,
  electrical offset; replaces the watch-window inject path).
- Generating the table directly from the shared X-macro (bindings make it non-trivial).
