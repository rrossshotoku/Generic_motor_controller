# ADR-013: Shared inter-MCU interface contract (OD + SPI) defined as a standalone package

## Status

Accepted

## Date

2026-06-21

## Context

The motor-control MCU exposes its object dictionary over SPI to the network MCU
(`../Lightweight_CMC`, STM32G431), which in turn exposes it on an Ethernet bus; the PC tooling
connects to *that*, not to this MCU directly (confirms ADR-001's two-MCU split). The SPI link is
therefore a real boundary between two separate firmware codebases. The protocol framing and OD
must be agreed **before** either side implements them, and must not drift.

## Decision

Define the boundary as a **shared, dependency-free interface package** that both firmwares (and
host tools) include, located at **`../Lightweight_CMC/Interface/`**:

- `mc_if_protocol.h` — SPI wire format: fixed 64-byte frame, CRC16/Modbus header+payload,
  message types, and **all** payloads including the ones spec 06 left open (OD read/write
  responses, heartbeat, error) plus OD-result and protocol-error code enums and node states.
- `mc_if_od.h` — OD data-type/access enums, scaling constants, control/status bits, and the
  canonical `MC_IF_OD_OBJECTS(X)` X-macro object list (each side generates its table from it).
- `INTERFACE_SPEC.md`, `README.md` — the human contract + rationale.

Framing was **redesigned** for this link (user approved): fixed-size full-duplex 64-byte frames
(trivial, self-synchronising SPI-slave DMA), network MCU = master / motor MCU = slave, SPI mode
0 8-bit MSB-first, pipelined OD responses correlated by sequence, cyclic CMD/STATUS as the
real-time channel. Manufacturer objects (0x2xxx: gains, telemetry, test-injection) are
**FLOAT32 SI** (exact, ideal for tuning/graphing); CiA-402 standard objects (0x6xxx) stay
scaled-int with documented factors.

## Reasoning

A single source of truth shared by both repos is the only robust way to keep a two-codebase
boundary in sync. Fixed-size frames suit an SPI slave far better than length-negotiated ones.
Float32 manufacturer objects remove scaling ambiguity exactly where tuning needs precision.

## Consequences

- This project's `mc_spi_protocol.{h,c}` should be **reconciled** to include the shared header
  (drop duplicated struct/enums; keep encode/decode + transport). The shared sync word, version,
  CRC, and cyclic payloads already match the existing implementation.
- The OD implementation (`mc_od` + object table) is generated from `MC_IF_OD_OBJECTS`, mapping
  each entry to a live variable / shadow config / callback.
- The network MCU implements the master side from the same headers.
- The shared package is itself a source-of-truth document; changes bump `MC_IF_PROTOCOL_VERSION`.

## Files affected

- ../Lightweight_CMC/Interface/{mc_if_protocol.h, mc_if_od.h, INTERFACE_SPEC.md, README.md}
- docs/spec/06_spi_protocol.md, docs/spec/05_object_dictionary.md (point to the shared contract)
- (next) include/mc_spi_protocol.h reconcile; src/mc_od.c object table from the X-macro

## Open questions

- CiA-402 scale factor values (proposed power-of-ten SI); cyclic rate; command-timeout; SPI clock.
- Configurable PDO mapping vs the fixed CYCLIC_STATUS set (v1 = fixed set + OD polling).
- How the two repos physically share the folder (relative include path, copy, or submodule).
