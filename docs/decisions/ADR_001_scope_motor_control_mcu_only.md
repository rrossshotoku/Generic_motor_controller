# ADR-001: Scope — motor-control MCU only; freeze the inter-MCU SPI seam

## Status

Accepted

## Date

2026-06-19

## Context

The system is a two-MCU design: a network MCU exposes external comms and its own
object dictionary, and a motor-control MCU owns real-time control. The external network
protocol is `not_fixed`, and no network-MCU firmware exists in either repository. Board
GPIOs (`WIZ_RSTn`, `EXTI_WIZ`) hint at a WIZnet/Ethernet controller, but the topology is
undefined.

## Decision

This project builds the **motor-control MCU firmware only**. The inter-MCU SPI link is
treated as a **frozen boundary contract** and must be fully specified (framing, CRC, and
all message payloads). The network MCU, its object dictionary, and the external protocol
are out of scope and handled as a separate project.

## Reasoning

Concentrates effort on the real-time control re-architecture, which is where the value and
risk sit. A fully specified SPI seam lets the two MCUs be developed independently. The
cyclic command/status payloads are already defined; only the non-cyclic payloads need
completing.

## Consequences

- SPI protocol: must define `OD_READ_RESP`, `OD_WRITE_RESP`, `HEARTBEAT`, and `ERROR`
  payload structures (cyclic command/status already exist).
- The motor-control MCU OD is in scope; the network-MCU OD is not.
- Inter-MCU SPI role on this board is **slave** (SPI2), matching the reference board.
- No external-protocol (CANopen/Ethernet) work in this project.

## Files affected

- docs/spec/06_spi_protocol.md
- include/mc_spi_protocol.h
- requirements.yaml

## Open questions

- Final inter-MCU framing constants (sync word value, CRC poly/init) — confirm against
  `mc_spi_protocol.c` once reviewed.
- Cyclic exchange rate and command-timeout value (drives the SPI-timeout fault).
