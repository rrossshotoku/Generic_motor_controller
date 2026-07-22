# ADR-067: Persistent position recall (last-position storage) for non-back-drivable incremental axes

- Status: Accepted (implemented — memory map + journal module + runtime + OD + GUI)
- Date: 2026-07-21
- Related: ADR-064 (bootloader flash map), ADR-010 (NV parameter store), ADR-057 (homing / NOT_HOMED), ADR-033/REQ-0013 (movement_status), ADR-052 (incremental quad encoder)

## Context

An **incremental** (quad) encoder loses its absolute position at power-off, so normally the axis
must re-home each power-up (NOT_HOMED). For an actuator that **physically cannot move while
powered off** (not back-drivable), the position at power-up equals the position at power-down —
so if the last position was reliably saved, homing can be skipped. Absolute (SSI) axes never
need this (they self-locate).

## Decision

Persist the position after every completed move, guarded by a validity flag, and on boot recall
it in place of homing — but only for an incremental encoder with the feature enabled.

**Motion signal:** reuse the existing `movement_status.MC_IF_MOVE_MOVING` (motor-authoritative,
already streamed) — NOT a new statusword bit (that would duplicate it and collide with
`0x6041` TARGET_REACHED semantics). `MOVING` is instantaneous, so the recall adds its own
**settle dwell (~1 s)** on top: store only once `MOVING` has stayed clear for the dwell.

**Runtime (slow/supervisory context only — no fast/medium-loop flash writes):**
- On `MOVING` rising edge → write an **INVALID** marker to the journal (the stored position is
  now stale; a mid-move power loss must land on "invalid").
- Once `MOVING` has been clear for the settle dwell → write the current position with a **VALID**
  marker (store on change only, to limit wear).

**Startup:** if `position_recall_enable` and the encoder is incremental and the latest journal
record is VALID (good CRC) → load the stored position as the mechanical zero / anchor, set
`s_homed = true`, do **not** raise NOT_HOMED. Otherwise (feature off, absolute encoder, no
record, or latest record INVALID = power lost mid-move) → normal NOT_HOMED, require homing.

## Flash map (done this step) — ADR-064 map, revised

Dedicated **journaled** region, separate from the config store; both preserved across OTA
updates with **no bootloader change / no reflash** (see the note below the table):

| Region | Address | Pages | Purpose |
|---|---|---|---|
| app (FLASH) | `0x08008800`–`0x0807DFFF` | b1 17–127, b2 0–123 | firmware (470K, was 474K) |
| **POS_RECALL** | `0x0807E000`–`0x0807EFFF` | **b2 124/125 (4K)** | position-recall journal (new) |
| CONFIG | `0x0807F000`–`0x0807FFFF` | b2 126/127 (4K) | NV parameter store (ADR-010) |

- `STM32G474RETX_FLASH.ld`: FLASH 474K→470K; **named `POS_RECALL` + `CONFIG` MEMORY regions**
  (so both show in CubeIDE) + `_pos_recall_origin` / `_pos_recall_length` linker symbols.
- `boot/boot_flash.c`: **unchanged — no bootloader reflash needed.** The OTA erase is
  *image-sized* (ADR-065: only the pages the image occupies), and the linker caps the app at
  470K (ends at bank2 pg123), so no image can ever reach the journal at pg124/125. Config is
  preserved by the existing erase-range exclusion of pg126/127; the journal by the app-size cap.
  (Relies on the deployed bootloader's image-sized erase, which it has.)

## Journal design (implemented)

A wear-levelled journal in POS_RECALL (2 pages, 256 fixed 16-byte records
`{seq (u32), position_rad (f32), marker (VALID/INVALID), crc32}`), split across two modules to
keep the architecture's HAL-free/boundary seam:

- `mc_pos_recall.c` (HAL-free, host-compilable) — the log logic: records are appended linearly
  slot 0→255; the highest-`seq` good-CRC record is authoritative; on wrap both pages are erased
  and the log restarts at slot 0 (only the latest record is ever needed). CRC32 (reflected,
  0xEDB88320) matches `mc_persistent_store` / `boot_flash`. Erased slots read `seq == 0xFFFFFFFF`
  and are skipped — the same erased-flash-tolerant read pattern the config store already uses.
- `mc_pos_recall_port_stm32g474.c` (boundary) — the flash ops (read via memcpy, program
  doubleword, erase-all), located by the linker symbols `_pos_recall_origin`/`_pos_recall_length`.

**Flash-write context:** the invalidate write fires on the MOVING rising edge — i.e. inherently
while the drive runs — so these writes are **not** gated on the power stage being off (unlike the
config store). Safe because POS_RECALL is in bank2: read-while-write against the bank1-resident
hot ISR code, the same basis the config store relies on. The store-on-settle write happens after
the settle dwell; store-on-change (skip if the settled position moved < 1 mrad) limits wear.

## Runtime (implemented, `mc_scheduler.c`)

- **Boot:** `MC_PosRecall_Init()` scans the journal in `MC_Framework_Init`. A one-shot in the
  medium loop (after the first good sample, before the drive is enabled) adopts the stored
  position **iff** `position_recall_enable` and the encoder is incremental and the latest record
  is VALID: `s_home_offset_rad = position − stored_P`, `s_homed = true` (skips NOT_HOMED).
- **Slow loop:** `pos_recall_service_slow()` runs only when enabled, incremental, **and homed**
  (an un-homed position must never be journalled). MOVING rising edge → `MarkMoving()` (INVALID);
  MOVING clear for the settle dwell (100 × 100 Hz ticks = 1 s) → `Store()` (VALID).

## OD (added, shared contract, additive — no version bump)

- `0x2700:11 position_recall_enable` (U8 RW PERSIST) — feature on/off; inert on absolute encoders.
- `0x2700:12 position_recall_status` (U8 RO) — 0 off/N-A / 1 recalled-valid (homing skipped) /
  2 stale→homing required / 3 enabled-but-nothing-stored. Lets the operator see what happened at
  boot. (Homing block, since it substitutes for homing.)

## Consequences

- Frequent moves are handled without wearing the config store (dedicated journal + store-on-change).
- Safety hinges on the INVALID-on-move-start write landing before motion is trusted; a mid-move
  power loss therefore reverts to NOT_HOMED (fail-safe). The `!s_homed` gate ensures a bogus
  (un-homed) position is never stored VALID.
- **Follow-ups:** a golden-reference test (store → power-cycle → recall matches; INVALID mid-move
  → NOT_HOMED). Consider also invalidating on fault/OC/brown-out mid-move, and a per-board
  back-drivable flag so the feature can't be enabled where it is unsafe.
