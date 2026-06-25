# ADR-038: Harden the startup position anchor against the cold-boot home-load race

- **Status:** Accepted
- **Date:** 2026-06-25
- **Supersedes the mechanism of:** ADR-037 (same goal — nearest-turn startup position; this changes *how*)
- **Related:** ADR-022 (mechanical home), ADR-010 (flash persistence), ADR-008 (state estimation)

## Context

ADR-037 added a **one-shot** startup seed: on the *first valid encoder sample*, anchor
`continuous = s_home_offset_rad + wrap_pi(single − s_home_offset_rad)` and latch `s_pos_seeded`.

A user reproduced a residual bug **only on a true cold boot** (unplug power), never on a soft/debugger
reset, with the rotor against a physical end-stop:

- Stops at `−0.1078` and `−6.1691`; mech zero set at the midpoint `−3.138` → symmetric ±3.03 arc.
- Move to `−3.0306`, power-cycle → `0x6064` reads **`+3.2527`** (`= −3.0306 + 2π`, i.e. one turn off, **> π**).
- The **zero point itself still read 0** after a cold boot, so the home value *was* persisted correctly.

The numbers are an exact fingerprint of **"the seed ran while `s_home_offset_rad` was still 0"**:

| start | `single` | seed with home=0 → `continuous` | then home loads −3.138 → `0x6064 = continuous − home` |
|---|---|---|---|
| zero point | 3.145 | `wrap_pi(3.145) = −3.138` | `−3.138 − (−3.138) = 0` ✅ (looks saved) |
| −3.0306 | 0.114 | `wrap_pi(0.114) = 0.114` | `0.114 − (−3.138) = +3.2527` ❌ |

Both observations fall out exactly. The seed anchored the wrap to **0** (the encoder zero) instead of the
loaded home, so it cancels *only* at home and is ~1 turn off elsewhere. A **soft reset hid it**: RAM is
retained, so `s_pos_seeded` stayed `true` and `continuous` kept its good value — the seed never re-ran, so
its wrap was never actually exercised. The cold boot is the first real test of the seed, and it lost the race.

The source order *looks* safe (`MC_Framework_Init` loads home at `mc_scheduler.c:390`, **before**
`HAL_TIM_Base_Start_IT(&htim7)` at `main.c:118`, so the TIM7 seed should see the loaded home). The fact that
the symptom still appears means a one-shot that depends on *ordering* is too fragile to trust on real
hardware (stale build, encoder-ready timing, or any future reorder all defeat it silently).

## Decision

Replace the one-shot seed with a **continuously self-correcting anchor**:

- While the drive has **never been enabled** (`!s_pos_locked`), re-anchor **every** medium cycle from the
  live absolute reading: `continuous = s_home_offset_rad + wrap_pi(single − s_home_offset_rad)`.
  This is **idempotent** once correct and **self-correcting** if `s_home_offset_rad` is loaded late, if the
  encoder is slow to read on a cold boot, or if home is re-set — the anchor always reflects the *current* home.
- **Lock on the first drive-enable** (`s_eff_drive` → `s_pos_locked = true`). After the lock the estimator's
  delta accumulation owns the continuous position, so motion tracks **true multi-turn** (deltas accumulate
  past ±π without being wrapped back). Once locked it stays locked for the power cycle (a later disable does
  **not** re-anchor — that would risk a jump if the axis moved > ½ turn).

Because the difference between the wrong (home=0) and right anchor is always a whole number of turns, the
re-anchor only ever shifts `continuous` by ±2π — fractional position, finite-difference velocity, and the
observer are undisturbed (`SeedContinuous` also moves `obs_theta`, so no phantom following error).

**Build marker.** Added `g_mc_debug.fw_build` (set to `38` in `MC_Framework_Init`). The "is the right image
actually flashed?" question recurred through this whole investigation; reading one watch value now answers it.

The hardware assumption is unchanged from ADR-037: a single-turn absolute encoder can only place the axis
within **±½ turn of home** across a power cycle. The robust quadrature + bottom-stroke-homing path
(re-home on startup) remains the real fix for unlimited range.

Motor-only — no contract change, **no `MC_IF_PROTOCOL_VERSION` bump**.

## Consequences

- Cold-boot startup position is correct across the whole arc, not just at home — independent of the
  home-load / encoder-ready timing that defeated the one-shot seed.
- While disabled, `0x6064` always reflects the nearest-turn home-relative position (good for the user's
  ±3.03 arc, which sits safely inside ±π); on the first enable it freezes and tracks multi-turn.
- Minor: while *disabled*, hand-moving the rotor shows finite-difference velocity but ~0 observer velocity
  (the anchor resets `obs_theta` each cycle). Disabled velocity is informational only — acceptable.
- `fw_build` lets the operator confirm the flashed firmware at a glance.

## Files

`src/mc_scheduler.c` (anchor logic + lock latch + `fw_build`), `include/mc_debug.h` (`fw_build` field);
`ADR_000_decision_log.md`, `requirements.yaml`, `docs/spec/11_feedback_and_encoders.md`.
