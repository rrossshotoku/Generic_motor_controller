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
