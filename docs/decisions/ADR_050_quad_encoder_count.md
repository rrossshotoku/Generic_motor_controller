# ADR-050: Quadrature encoder count exposed (TIM2)

- Status: Accepted
- Date: 2026-06-29
- Related: ADR-004 (per-board config; "quadrature later"), ADR-038 (encoder/anchor), the
  position-feedback interface

## Context

Quadrature encoders are wired to **TIM2** (PA15 = CH1, PB3 = CH2). CubeMX configures TIM2
correctly as a **4× (TI12)** encoder with a 32-bit free-running counter and an input filter — but
the encoder was **never started** (no `HAL_TIM_Encoder_Start`), so `TIM2->CNT` did not move. For
bring-up the operator wants to read the raw count from the PC tool to verify wiring and direction,
ahead of folding quadrature into the position-feedback interface (the planned long-term encoder).

## Decision

- **Start the encoder at boot:** `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)` in `main.c`
  USER CODE 2 (alongside the other peripheral starts).
- **Read via a thin STM32 boundary** (`mc_quad_encoder_stm32g474.c`,
  `MC_QuadEnc_Count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2)`) so the HAL-free scheduler stays
  HAL-free.
- The scheduler **mirrors it each medium cycle to OD `0x2510:4 quad_encoder_count`** (I32, RO,
  PDO) for a manual read (or graph) from the PC tool. Signed around the power-on zero
  (negative = reverse).

This is a **raw diagnostic count** for bring-up; it is **not yet** the position-feedback source.

## Consequences

- Additive OD entry (`0x2510:4` is PDO-mappable but not in the default cyclic frame) → no
  `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected (transport). Logged in CHANGELOG.
- New boundary module (`mc_quad_encoder_stm32g474.c`) + header; the IDE globs `src/`.
- The 32-bit counter free-runs; turning it into a turns+fraction position (wrap tracking, scale by
  encoder PPR×4, direction sign, and routing through the position-feedback interface in place of /
  alongside the SSI) is the deferred next step.
