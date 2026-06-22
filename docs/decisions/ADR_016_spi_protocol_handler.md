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

## Update (on-target): re-arm priority + pipelined double-buffer

Two further on-target findings, fixed together:

1. **Re-arm raced the control loop (re-arm fails in ~600 ms bursts).** The SPI2 DMA IRQ sits at
   priority 3, *below* the 1 kHz medium loop (TIM7 = priority 2). When the beat between the
   master's ~1 kHz frame rate and TIM7 drifted them into phase, the medium loop (blocking SSI
   read + control cascade) preempted and delayed the per-transaction re-arm, which then missed
   the master's next frame — a burst of fails once per beat period (~600 ms). Raising the SPI2
   DMA/IRQ to priority 1 (above the medium loop) was tried and **reverted**: putting a comms ISR
   above the velocity loop inverts the proper hierarchy (control must always preempt comms) and
   adds bounded but real jitter (~25 µs, CRC-dominated) to the loop. **Resolution:** keep SPI2 at
   priority 3 and rely on the pipelined double-buffer (item 2) to make the re-arm prompt — the
   time-critical work is now a ~1-2 µs swap+arm, which fits the master's inter-frame gap even when
   the medium loop delays the callback. (Decision: control-loop primacy over comms; 2026-06-22.)

2. **Re-arm latency scaled with handler work (fails + resets when the telemetry map filled).**
   With the original single buffer the re-arm happened *after* `MC_Comms_HandleTransaction`
   (frame validate + OD apply + telemetry gather). Subscribing N PDOs made the gather longer and,
   more importantly, the **master bursts frames back-to-back** when its own main loop is delayed
   (its `cia402_tick` catches up by firing several 1 ms ticks with no inter-frame gap, e.g. while
   forwarding telemetry over UDP). A slave that re-arms only after a long handler cannot keep up
   with a zero-gap burst → desync → fails + resets. Fix: **pipelined double-buffer.** Two TX
   buffers (`s_armed`, `s_prepared`); on transfer-complete: swap → **re-arm immediately** with the
   already-prepared frame (just a pointer swap + arm, ~1-2 us) → *then* run the handler to fill the
   freed buffer for the transaction after next. Re-arm latency is now independent of handler work,
   so the slave tolerates small / back-to-back gaps.

**Contract impact:** the pipelined double-buffer makes an OD response land **two** transactions
after its request instead of one. This is **not** a wire/OD change — the master already correlates
responses by `sequence` (and a 100 ms response timeout), so any pipeline depth is transparent and
no `MC_IF_PROTOCOL_VERSION` bump or `Interface/CHANGELOG.md` entry is required. Recorded here for
the network-MCU author's awareness; no action needed their side. Telemetry is likewise one frame
(~1 ms) older — negligible for 1 kHz graphing/tuning.

**Master-side item (network MCU — tracked in the Lightweight_CMC repo, not a contract change):**
with the SPI2 priority reverted, the pipelined double-buffer absorbs *small* inter-frame gaps, but
a true zero-gap burst from the master can still cause occasional re-arm fails (the callback may be
delayed by the medium loop). The root cause is the master's tick catch-up
(`cia402_tick`: `s_last_tick_ms += CYCLE_PERIOD_MS` after a stall) emitting frames with no gap —
fragile in general. The proper fix is master-side: clamp the catch-up (`s_last_tick_ms = time_ms()`
after a long stall, or send at most one frame per main-loop pass) so the 1 ms cadence survives a
main-loop hitch (e.g. a blocking W6100 send). Being addressed separately by the network-MCU author;
no motor-side action.

**Resolved 2026-06-22 (master-side).** The network MCU removed the catch-up: `cia402_tick`
(`Lightweight_CMC/app/cia402/cia402.c`) now does `s_last_tick_ms = now_ms` — after a main-loop
stall it sends one frame and resumes the 1 ms cadence (drift-tolerant), never a zero-gap burst.
So the master always presents gapped frames at the cyclic rate, which the pipelined re-arm at
priority 3 handles with control-loop primacy intact. This is the agreed **split**: the slave is
*not* designed to absorb true zero-gap bursts (that would require inverting the priority hierarchy,
rejected above); instead the master guarantees the gap. Cross-project record: `Interface/REQUESTS.md`
**REQ-0007** — its original "what's needed" (raise the re-arm ISR above the loops; sustain zero
re-arm gap) is superseded by this split; its acceptance bullet 3 ("back-to-back **at the cyclic
rate**") is the behaviour actually delivered. End-to-end acceptance (apply a 16-PDO map, zero
re-arm-fail lines) pending on-target verification.

`g_spi_slave` gains `last_rearm_hal` (HAL status of the last re-arm: 0=OK, 1=ERR, 2=BUSY,
3=TIMEOUT) to diagnose any residual failures from the watch window.
