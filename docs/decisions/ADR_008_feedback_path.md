# ADR-008: Feedback path — SSI encoder read, decode, and state estimation

## Status

Accepted

## Date

2026-06-19

## Context

The framework needs mechanical position/velocity and the electrical angle for FOC, from the
SSI absolute encoder. The proven scheme in `bldc_axis_controller` was confirmed from the SPI
register config and the decode code (not the contradictory header comments).

## Decision

- **SSI read** (`mc_ssi_encoder_stm32g474.c`): SPI1 master, 16-bit, **CPOL=1/CPHA=0 (Mode 2)**,
  MSB-first, ~1.33 MHz; two 16-bit words → 32-bit frame; **position = bits [30:10]** (21-bit,
  binary, no Gray). Blocking `HAL_SPI_TransmitReceive` (~24 µs) in the **1 kHz medium loop**.
- **Decode** (`mc_ssi_encoder.c`, HAL-free): extract the position field, apply **direction**
  (board: invert) and the mechanical zero offset → a single-turn mechanical angle.
- **State estimator** (`mc_state_estimator.c`, HAL-free): multi-turn **continuous position**
  via wrap detection; **mechanical velocity** by finite-difference + first-order low-pass
  (cutoff from config, ~20 Hz); **electrical angle** = `wrap(single · pole_pairs + offset)`.
- The **position-tracking velocity observer** (ADR-003 default) is deferred to **B2b** so its
  gains can be tuned on the bench; finite-difference is the B2 velocity path.
- The encoder is read in the medium loop; supplying the electrical angle to the 20 kHz fast
  loop for FOC (interpolation/prediction) is handled in **D1**.

## Reasoning

Matches the validated SSI frame and scheme. A 1 kHz read is ample for the estimator and for
hand-turn bring-up; a blocking read is simple and adequate (DMA can come later). Keeping the
decode and estimator HAL-free preserves boundary isolation and host-testability.

## Consequences

- New `mc_ssi_encoder.c`, `mc_ssi_encoder_stm32g474.c`, `mc_state_estimator.c`; the estimator
  struct gains continuous-position / prev-single / filtered-velocity state.
- The scheduler runs read + estimate in the medium loop and mirrors raw position, continuous
  position, velocity, electrical angle, and validity to `g_mc_debug`.
- Direction inversion and the electrical/zero offsets are fixed here (offsets 0 until
  calibration/alignment in Phase E).

## Files affected

- include/mc_ssi_encoder.h, include/mc_state_estimator.h
- src/mc_ssi_encoder.c, src/mc_ssi_encoder_stm32g474.c, src/mc_state_estimator.c
- include/mc_debug.h
- src/mc_scheduler.c
- docs/spec/11_feedback_and_encoders.md

## Open questions

- Observer gains (B2b, on-bench tuning).
- Acceleration estimation (not done in B2).
- Fast-loop electrical-angle interpolation/prediction for FOC (D1).
- Optionally move the SSI read to DMA if the medium-loop blocking time becomes a concern.
