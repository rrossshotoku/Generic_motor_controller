# Persistent Parameter Store

## First implementation

Use a simple flash-backed parameter store for selected configuration and calibration values.

Required features:

- version field
- structure size field
- CRC
- defaults
- save command
- load on boot
- factory reset command

Not required initially:

- wear levelling
- multiple profiles
- rollback
- migration between arbitrary versions
- event history

## Storage contents

- motor parameters
- encoder configuration
- controller gains
- trajectory limits
- current limits
- fault thresholds
- calibration results
- soft limits

## Execution rules

- No flash write in fast or medium loop.
- Save request is latched from OD and executed in slow loop.
- If CRC/version invalid on boot, load defaults and report parameter-store warning.

## Realized architecture (see ADR-010)

Two layers:

- **Generic NV store** (`mc_persistent_store`): a reserved flash region at the top of flash
  (carved out in the linker script). One record = `header { magic 'MCPF', version, size_bytes,
  crc32 }` + `payload`. Validates magic/version/size/CRC32 on load. **A/B two-page ping-pong**
  with a sequence counter so a power loss mid-write keeps the last-good copy. Saves are latched
  and written only in the slow/background context (HAL_FLASH unlock → page erase → program
  double-words → lock). STM32G474: 2 KB pages, 64-bit programming.
- **Schema layer** (`mc_params`): `MC_Params_t` is the payload — the pointer-free module config
  structs (board scaling, motor model, SSI config, estimator + observer gains, controller PID
  configs, limits, soft limits, fault thresholds) plus calibration results (electrical offset,
  current offsets, mechanical zero, phase order). No runtime pointers are ever stored.
  `LoadDefaults` (from the compile-time profile loaders), `CaptureFromLive` (snapshot before
  save), `ApplyToLive` (apply after load / factory reset).

Boot: read NV → valid ? apply stored : apply defaults + warning. Save: latch → slow-loop
capture + write. Factory reset: invalidate NV → defaults next boot. Version/CRC mismatch →
defaults (no migration initially).

The live module configs must be reachable by capture/apply, so they move to a small axis/config
registry (currently statics in `mc_scheduler`).

## Implementation (ADR-023, extended in ADR-044)

The flash payload is `MC_Params_t` = the calibration subset (`MC_CalibData_t`: electrical offset,
current offsets, mechanical zero, phase order) **plus a serialized blob of every PERSIST OD entry**
(gains / limits / motor model / board), gathered by `MC_Od_GatherPersistent` as
`{index, sub, len, value}` records and restored by `MC_Od_RestorePersistent` into `g_od`. NV region =
bank 2 pages 126/127 (0x0807F000/0x0807F800), reserved in the linker. Alignment capture and the OD
save command (`0x2800:1`) latch a save; the slow loop writes it when the power stage is off; boot
reloads and applies it.

The blob buffer is `MC_PARAMS_OD_BLOB_MAX` = **448 B** (ADR-044). It MUST stay ≥ the total PERSIST
size (~318 B currently): `MC_Od_GatherPersistent` **silently drops** entries past the cap — at the old
256 B it truncated `0x2600:6/7`, `0x2700:3/4`, and `0x6081-5`. Re-check the cap (and keep
`sizeof(MC_Params_t) ≤ MC_PARAM_STORE_MAX_PAYLOAD`). Because the blob is a self-describing TLV keyed
by `{index, sub}`, **adding** a PERSIST entry is backward-compatible: an old record simply lacks the
new key, which then keeps its default — so no `MC_PARAM_STORE_VERSION` bump is needed for additions
(bump only when a field's type/meaning changes or `MC_Params_t` grows past the buffer).

## Position-recall journal (ADR-067)

A **second, independent** flash store for the last-position feature on non-back-drivable incremental
axes: `POS_RECALL` = bank 2 pages 124/125 (`0x0807E000`, 4 KB), a named linker MEMORY region located
by `_pos_recall_origin`/`_pos_recall_length`. It is **not** the A/B config store — it is a
wear-levelled append journal of 256 fixed 16-byte records `{seq, position_rad, marker, crc32}`
(`mc_pos_recall.c` logic + `mc_pos_recall_port_stm32g474.c` flash ops). The highest-`seq` good-CRC
record wins; on wrap both pages are erased and the log restarts. Unlike the config store, its writes
are **not** gated on the power stage being off (the invalidate-on-move-start write fires while the
drive runs) — safe via bank-2 read-while-write against the bank-1 hot code. Both regions survive OTA
with no bootloader change: the OTA erase is image-sized and the app is linker-capped at 470 KB (ends
at bank 2 pg123), so an update never reaches pg124–127.
