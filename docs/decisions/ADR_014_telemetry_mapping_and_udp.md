# ADR-014: Configurable runtime telemetry mapping + all-UDP network exposure

## Status

Accepted

## Date

2026-06-21

## Context

The PC tuning/graphing tool needs to stream a chosen set of live variables and change that set
at runtime, reaching the data via the network MCU over Ethernet. Per-variable OD polling is too
inefficient for streaming; a fixed telemetry struct can't graph arbitrary signals without a
reflash. User decisions: ~12 channels @ ~1 kHz, 64-byte SPI frame, configurable mapping,
scope-buffer later, and the network MCU does **everything over UDP**.

## Decision

- **Configurable telemetry map (CANopen TX-PDO style)** on the motor MCU OD at **`0x2A00`**:
  sub0 = count, sub1..16 = U32 entries `(index<<16)|(sub<<8)|bitlen`. The cyclic telemetry frame
  becomes a fixed 12-byte header (`MC_IfCyclicStatusHeader_t`: statusword, mode, node_state,
  error_code, `map_version`, `map_byte_count`, status_counter) + a mapped blob (≤ 40 B ≈ 10
  float32) holding the mapped OD values, packed LE in map order.
- **Runtime reconfiguration:** the host rewrites `0x2A00` live (deactivate→edit→activate); the
  slave validates (entries exist, `MC_IF_F_PDO`, total ≤ 40 B), builds a gather list, swaps it in
  **atomically** (double-buffer), and bumps `map_version`. Each frame carries `map_version` so the
  host re-columns safely across the transition. The standard actuals (0x6064/0x606C/0x6077) are
  ordinary mappable entries (recommended default map lists them first).
- **Network side = UDP only** (`NETWORK_UDP_SPEC.md`): the network MCU bridges OD-over-UDP ⟷
  OD-over-SPI (request/response, PC retransmits on timeout) and pushes a fire-and-forget telemetry
  stream (batched, carrying `map_version` + per-sample `status_counter`). The PC chooses graphed
  signals by writing `0x2A00` over UDP (bridged to SPI) — end-to-end runtime selection.

## Reasoning

Bundling the chosen signals into the periodic frame is far more efficient than polling (~10
float32 per 1 kHz frame ≈ 10% of a 6 MHz SPI link). Configurable mapping makes the tuning loop
fully live (pick signals, change gains, fire steps, watch) with no reflash. UDP-only keeps the
network MCU simple; app-level retransmit covers OD writes, and telemetry tolerates loss.

## Consequences

- Motor MCU: OD gains the `0x2A00` array object + a validate/gather/atomic-swap; the SPI-slave
  telemetry builder emits header + mapped blob each cycle (comm/slow context, not the fast loop).
- Network MCU: implements `NETWORK_UDP_SPEC.md` (UDP OD bridge + telemetry forwarder/batcher).
- The PC speaks OD (index/sub/type/scaling) end-to-end; the OD is the single shared model.
- Scope-buffer (20 kHz triggered capture) is deferred (separate future ADR).

## Files affected

- ../Lightweight_CMC/Interface/{mc_if_od.h, mc_if_protocol.h, INTERFACE_SPEC.md, NETWORK_UDP_SPEC.md, README.md}
- docs/spec/05_object_dictionary.md, docs/spec/06_spi_protocol.md (telemetry map note)
- (next) src/mc_od.c (0x2A00 + gather), SPI-slave transport telemetry builder

## Open questions

- Final UDP ports / endpoint discovery; whether the CMC exposes its own config OD range.
- Default telemetry map contents.
- Cyclic rate / command-timeout / SPI clock (shared with ADR-013).
