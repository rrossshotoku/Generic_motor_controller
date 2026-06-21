# ADR-016: SPI inter-MCU protocol handler + SPI2-slave DMA (F2a + F2b)

## Status

Accepted

## Date

2026-06-21

## Context

Putting the OD on the wire for the network MCU. The shared contract
(`../Lightweight_CMC/Interface/`) defines fixed 64-byte full-duplex frames, CRC16/Modbus,
cyclic command/telemetry, OD read/write, and the configurable 0x2A00 telemetry map. The
implementation splits into the HAL-free protocol logic (this ADR, F2a) and the SPI2 DMA boundary
(F2b).

## Decision

- **`mc_comms.c` (HAL-free)** builds **against the shared headers** (`mc_if_protocol.h`,
  `mc_if_od.h`) — the single source of truth — so the motor MCU and network MCU agree by
  construction. (The motor build adds `../Lightweight_CMC/Interface` to its include path.)
- **Per-transaction handler** `MC_Comms_HandleTransaction(rx, tx_next)`: validate the 64-byte
  frame (sync/version/header-CRC/payload-CRC); dispatch by type:
  - `CYCLIC_CMD` → apply (controlword enable, joystick/profile-velocity → velocity demand);
  - `OD_READ_REQ` → `MC_Od_ReadRaw` → `OD_READ_RESP`;
  - `OD_WRITE_REQ` → `MC_Od_Write` (or the 0x2A00 map handler) → `OD_WRITE_RESP`.
  The response to request N is produced as `tx_next` (sent on transaction N+1, pipelined); the
  default `tx_next` is the cyclic telemetry frame.
- **Telemetry**: `build_telemetry` emits the 12-byte status header + the mapped blob, gathered
  from the **0x2A00 map** (owned by `mc_comms`, validated on activation: entries exist, are
  PDO-mappable, total ≤ 40 B; `map_version` bumped). `MC_Od_ReadRaw` was added to the OD engine.
- **Command dead-man**: `MC_Comms_CommandTimedOut()` (slow loop) trips after
  `MC_IF_COMMAND_TIMEOUT_MS` (30 ms) of no fresh cyclic command — but only **once a master has
  been seen** (inert during watch-window bring-up). On trip the scheduler zeroes the velocity
  demand (full quick-stop is the fault manager's job, E2).

## Reasoning

Keeping the protocol logic HAL-free and built on the shared headers guarantees both MCUs use the
identical wire format and OD map, and lets the handler be exercised from the watch window before
the DMA exists. The pipelined-response model matches the contract and suits an SPI slave.

## Consequences

- New `mc_comms.{h,c}`; `MC_Od_ReadRaw` added; scheduler calls `MC_Comms_Init` + the watchdog.
- **The motor project now depends on the shared `Interface` headers** (include path) — enforces
  single-source-of-truth (ADR-013).
- Cyclic-command apply is minimal (enable + velocity/jog) pending the mode manager (E1); the
  0x6xxx CiA-402 objects + scaling and the full command set wire up there.
- **F2b next**: the SPI2-slave DMA boundary (`mc_spi_slave_stm32g474.c`) — arm 64-byte
  full-duplex DMA, call the handler per transaction, re-arm — brought up on-target.

## Files affected

- include/mc_comms.h, src/mc_comms.c
- include/mc_od.h, src/mc_od.c (MC_Od_ReadRaw)
- src/mc_scheduler.c
- docs/spec/06_spi_protocol.md
- Interface/CHANGELOG.md [1.0.1] (operational defaults logged)

## Resolution (F2b — SPI2-slave DMA boundary)

`mc_spi_slave_stm32g474.c`: SPI2 as slave (8-bit, mode 0, hardware NSS) with RX/TX DMA. Two
64-byte buffers; `MC_SpiSlave_Init` (called from `main.c` USER CODE after the ADC start) loads
an idle telemetry frame and arms `HAL_SPI_TransmitReceive_DMA`. On `HAL_SPI_TxRxCpltCallback`
(SPI2 only — SPI1/SSI uses blocking transfers), it runs `MC_Comms_HandleTransaction(rx, tx)`
(which fills the next outbound frame) and re-arms. `HAL_SPI_ErrorCallback` recovers (abort +
rebuild idle + re-arm). The handler runs at the SPI DMA IRQ priority (below the 0/2 control
loops), does only bounded non-blocking work, and re-arms within the master's ~1 ms inter-frame
gap. `g_spi_slave` mirrors transaction/error counts. Single-buffer re-arm is adequate at 1 kHz;
revisit if errors appear on-target.

## Open questions

- On-target SPI2 bring-up: NSS resync robustness, re-arm timing margin.
- Mode-manager wiring of the full cyclic command + CiA-402 object scaling.

## Update (on-target): robust error recovery

First on-target run showed many SPI errors + re-arm failures (a cascade): the original error
handler only called `HAL_SPI_Abort`, leaving both DMA handles BUSY/locked so every re-arm fails.
Ported the proven `SPI2_Slave_Reset` from `bldc_axis_controller`: on any SPI/DMA error or failed
re-arm, abort + force-reset **both DMA handles** (State→READY, ErrorCode→NONE, unlock), clear
OVR/MODF/FRE flags, force `hspi2` READY/unlocked, then re-arm. `g_spi_slave` gains
`resets`/`err_overrun`/`last_hal_error`. NOTE: persistent overruns usually mean the *master's*
framing/timing is off (clock exactly 64 bytes, mode 0, NSS per frame, inter-frame gap) — the
reset keeps the slave resilient but the master must be disciplined.
