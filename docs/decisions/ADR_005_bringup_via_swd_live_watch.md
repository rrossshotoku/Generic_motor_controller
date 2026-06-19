# ADR-005: Bring-up and verification via on-target SWD live watch

## Status

Accepted

## Date

2026-06-19

## Context

The user prefers to bring the framework up incrementally on real STM32 hardware over SWD,
observing and exercising behaviour through the debugger's live watch window, rather than
investing in an extensive host (Unity) test suite up front. The reference firmware already
worked this way (volatile debug-mirror structs such as `sensors_debug`, a variable registry
in `comm_vars.c`, and DAC output of `iq` for scope viewing).

## Decision

Primary verification is **incremental on-target bring-up over SWD with live watch-window
inspection and command injection**. Host unit tests (Unity) are **optional/secondary**, used
only where a quick off-target check is convenient. ADR-002's golden-reference principle
stands: the old code remains the source of expected values/behaviour to compare against.

## Reasoning

Matches how the user works and how the proven controller was developed. Real hardware
exercises the timing-critical paths (ADC/PWM/SSI/ISR) that host tests cannot, and the live
watch window gives immediate observability without first building a comms stack.

## Consequences

- Each module exposes a `volatile` debug-mirror snapshot of key internals, plus a guarded set
  of `volatile` command/inject fields writable from the watch window. This is the bring-up
  command path *before* the OD / mode-manager path exists (echoes old `sensors_debug`,
  `comm_vars`, DAC `iq`).
- Bring-up build is debug-friendly (e.g. -Og/-O0, full symbols); watched and injected
  variables are `volatile` so they are neither optimised away nor cached.
- `mc_*` modules stay HAL-free and host-compilable anyway (boundary isolation), which keeps
  optional host tests possible later — but they are not a gate.
- Bring-up follows the hardware-safe order in spec 15: PWM disabled by default, current-offset
  calibration with PWM off, stationary encoder read, electrical-angle-vs-rotation check, then
  close current -> velocity -> position loops.
- First milestone: framework compiles and the 20 kHz / 1 kHz / slow loops run with debug
  mirrors visible and PWM in safe-off — observable in the watch window.

## Files affected

- docs/spec/15_basic_tests.md
- CLAUDE.md
- requirements.yaml

## Open questions

- Debugger/IDE for the watch window (STM32CubeIDE, Ozone, gdb) — affects watch/inject ergonomics.

## Resolved

- Debug-mirror convention: a **single shared `include/mc_debug.h`** with `g_mc_debug`
  (observe) and `g_mc_inject` (command, gated by `inject_enable`); modules contribute fields
  to these rather than each owning a separate debug struct. (Implemented in Stage A1.)
