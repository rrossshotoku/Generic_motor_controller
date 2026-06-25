# ADR-037: Startup nearest-turn position seed (single-turn encoder boundary)

- **Status:** Accepted
- **Date:** 2026-06-25
- **Related:** ADR-008 (state estimation), ADR-002 (clean-room vs golden reference); golden reference
  `../bldc_axis_controller/Core/Src/alignment.c`

## Context

The SSI encoder is **single-turn absolute** (21-bit = one revolution, no turn counter). On startup the
estimator anchors `continuous = single` (the `[0, 2π)` angle, `mc_state_estimator.c:63`), and the
home-relative position is `mech_position − s_home_offset_rad` (`mc_scheduler.c`), computed **without a
wrap**. `s_home_offset_rad` is a single-turn value captured at SET_MECH_ZERO. So when the start position
and home sit on **opposite sides of the encoder's 0↔2π seam**, the home-relative position is off by ~one
turn (~2π): the axis is right next to home but reports a full turn of phantom travel. (User-reported bug;
reproduced in a host test.)

The golden reference solves this in `alignment.c:238-252`: on startup it computes
`raw_delta = raw − stored_raw_zero` and **wraps it to ±CPR/2 (nearest half-turn)** before seeding the
continuous position. The new project was missing exactly this nearest-turn wrap.

## Decision

On the **first valid encoder sample** after startup, re-anchor the continuous position to the home-relative
position wrapped to the nearest turn:

```
home_rel   = wrap_pi(single − s_home_offset_rad)     // shortest distance from home, ∈ [−π, π]
continuous = s_home_offset_rad + home_rel            // so (mech − home_offset) = home_rel
```

so the startup home-relative position is the nearest-turn value; deltas then accumulate exactly as before
(multi-turn **within** a session is unaffected). One-shot (`s_pos_seeded`), wired into the scheduler's
first-sample path (it already has `s_home_offset_rad` and the single-turn sample). New estimator API
**`MC_StateEstimator_SeedContinuous`** sets `continuous_position_rad` + `obs_theta` + `mechanical.position`
and keeps `prev_single_rad` so the next delta stays small and the observer follows (no phantom error).

**Assumption (inherent to the hardware):** the axis is within **±½ turn of home at startup**. A single-turn
absolute encoder physically cannot recover the turn count across a power cycle, so this is the only correct
interpretation — and it is what the golden reference does. True multi-turn-across-power-cycle would need a
multi-turn encoder (separate decision).

Motor-only — no contract change, **no `MC_IF_PROTOCOL_VERSION` bump**.

## Consequences

- Startup position near the seam is correct (nearest-turn from home) instead of ~1 turn off.
- Multi-turn tracking within a session is unaffected.
- Complementary belt-and-braces: set `mechanical_zero_offset_rad` (currently 0) to push the encoder seam
  out of the working arc, so the working range never crosses it.

## Files

`include/mc_state_estimator.h`, `src/mc_state_estimator.c`, `src/mc_scheduler.c`;
`ADR_000_decision_log.md`, `requirements.yaml`.
