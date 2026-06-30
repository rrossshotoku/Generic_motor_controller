# ADR-052: Quad encoder → estimator (close the brushed velocity loop)

- Status: Accepted
- Date: 2026-06-30
- Related: ADR-003 (observer), ADR-050 (quad count), ADR-039 (brushed backend), the position-feedback interface

## Context

The brushed axis had no outer-loop feedback: the medium loop read only the SSI, which isn't present
on the brushed board, so the estimator never updated and the brushed motor ran current/torque only.
The quad count (ADR-050) is now wired; feeding it through the common position-sample interface into
the existing state estimator closes the brushed velocity loop with **zero changes** to the velocity
controller or its OD gains (the loop is encoder-agnostic — it consumes `s_est` velocity).

## Decision

- The medium loop builds an `MC_PositionSensorSample_t` from the quad: `position_rad = count *
  s_quad_rad_per_count`, where `s_quad_rad_per_count = 2π / quad_counts_per_rev` (`0x2500:8`, **signed**:
  the sign sets count direction). `absolute = false` (incremental — no absolute position until homed).
- It feeds the **same** `MC_StateEstimator_Update`; the estimator accumulates `wrap_pi` deltas, so the
  continuous quad value needs no single-turn anchor. The startup anchor (ADR-037/038) is now gated on
  `sample.absolute`, so SSI keeps it and the quad skips it.
- **Encoder source is tied to the backend for now (INTERIM):** brushed → quad, FOC → SSI. A clean
  per-board encoder-source selector is the next step (deliberately deferred).
- New config `0x2500:8 quad_counts_per_rev` (F32, RW, PERSIST), default 4000 (= 1000 lines ×4). All
  velocity-loop OD entries (`0x2300`) are reused unchanged.

## Consequences

- Additive OD entry → no `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected. Persist 376/448 B.
- Velocity mode works immediately on the brushed axis (incremental is fine for velocity). Absolute
  position mode + soft limits still need a **homing** routine (the quad has no absolute zero at
  power-on) — a later step.
- The estimator's electrical-angle output (`single * pole_pairs`) is meaningless for the continuous
  quad value, but the brushed backend ignores it. A future FOC-on-quad would need the single-turn
  modulo for commutation — out of scope here.
- INTERIM coupling: a FOC-on-quad or brushed-on-SSI build won't work until the encoder-source selector
  (next step) decouples them. Fine for the current two boards.

## Bring-up

Set `0x2500:8` to your encoder's 4× line count; if a velocity command runs away (positive feedback),
flip the sign. Watch `tlm_vel_actual` (`0x2310`) track the command and `quad_encoder_count` (`0x2510:4`) move.
