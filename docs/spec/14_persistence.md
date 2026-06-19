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
