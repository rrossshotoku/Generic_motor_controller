# ADR-002: Clean-room reimplementation validated against a golden reference

## Status

Accepted

## Date

2026-06-19

## Context

A proven single-axis controller exists (`../bldc_axis_controller`) with working FOC,
SVPWM, a position-tracking velocity observer, an S-curve planner, alignment, and fault
handling. The new framework is a clean-room re-architecture from the specs, but we do not
want to lose the correctness already achieved in those solved areas.

## Decision

Reimplement every module **clean-room from the specs** (not by copying old code), but
**validate each module numerically against the old implementation** as a golden reference
before trusting it. The old repo is also the authoritative source for proven numeric
constants (gains, offsets, thresholds, motor/board parameters).

## Reasoning

Clean-room yields a coherent, spec-aligned codebase with the new module boundaries, while
golden-reference validation keeps the re-derived algorithms honest in areas the old
controller already solved. This balances a clean result against re-introducing risk.

## Consequences

- Verification venue is on-target SWD bring-up with the live watch window (see ADR-005);
  host (Unity) tests are optional, not the gate. The golden-reference principle is unchanged
  — only where we observe it shifts to hardware.
- The old code remains the source of expected values/behaviour to compare against during
  bring-up: FOC transforms (Clarke/Park/SVPWM), the velocity observer, the S-curve planner,
  and current/velocity/position loop step responses.
- The HAL-free `mc_*` modules stay host-compilable (only the STM32 boundary modules touch
  hardware), which keeps optional host checks possible later.

## Files affected

- docs/spec/15_basic_tests.md
- tests/
- requirements.yaml

## Open questions

- Oracle strategy per module: port the old algorithm into a test fixture vs. capture I/O
  vectors from the running old firmware. Likely a mix; decide per module at implementation.
