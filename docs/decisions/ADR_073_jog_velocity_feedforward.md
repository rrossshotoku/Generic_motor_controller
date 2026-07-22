# ADR-073: Velocity feedforward for the position-integrated jog

- Status: Accepted
- Date: 2026-07-22
- Related: ADR-062 (position-integrated jog), ADR-031 (velocity feedforward / `velocity_ff_gain`),
  ADR-042 (velocity slew ramp), ADR-043 (soft-limit taper)

## Context

The position-integrated jog (`jog_position_mode = 1`, ADR-062) integrates the ramped stick velocity
into `s_pos_hold_rad` and lets the position cascade track it, so following-error / soft-limit / stiff-
hold protection apply while jogging. But the cascade runs with `s_traj.active = false`, so it takes
the "no active plan → hold the latched position" branch, which set **`v_ff = 0`**. The jog was
therefore **pure position feedback chasing a moving target** — the classic velocity-proportional
following lag (`≈ jog_vel / Kp`): the actual trails the reference while jogging and only catches up on
release. Real trajectory moves don't have this because the trajectory supplies `v_ff` (ADR-031); the
jog just never populated it.

## Decision

Feed forward the jog reference velocity through the existing `velocity_ff_gain` (0x2200:4) path.

- **The feedforward is the ACTUAL per-tick advance of the clamped reference**, not the raw `jog_vel`:
  `s_jog_ref_vel = (s_pos_hold_rad − ref_prev) / dt`, captured in the jog block *after* the leash
  (ADR-062) and soft-limit (ADR-043) clamps are applied. This matters: when the leash or a limit pins
  the reference, its advance — and thus the FF — collapses to **0**, instead of over-driving at
  `jog_vel` and defeating the leash. When unclamped it equals `jog_vel`.
- **One value, both cases.** `s_jog_ref_vel` defaults to 0 each medium tick and is set only by the
  position-jog block. The cascade's hold branch now uses `v_ff = s_jog_ref_vel`, so a genuine idle
  hold (jog block didn't run → still 0) stays pure feedback, while an active jog gets the FF. No
  separate "am I jogging" flag needed.
- **Same gain.** `s_eff_vel_cmd = s_vel_ff_gain·v_ff + vcorr` is unchanged — the jog FF flows through
  `velocity_ff_gain` exactly like the trajectory FF (per the request). Since the reference-delta *is*
  the true reference velocity, gain 1.0 (the default) is theoretically exact; a trimmed gain applies
  uniformly to jog and trajectory.

**No one-sample delay.** A one-sample preview (output `ref(k−1)`, FF = `Δref/dt`) would phase-align
the FF to the reference's next step and remove the residual `O(jog_vel·dt)` error, but that residual
is ~0.1 mrad at 1 kHz / jog speeds and costs a sample of jog latency — not worth it. Backward
difference (no delay) captures essentially all the benefit.

## Consequences

- The jog now tracks the stick with negligible following lag instead of a velocity-proportional
  offset; feel is tighter and position readout matches the commanded reference during the jog.
- Correct under the leash and soft limits by construction (FF = clamped advance), so it can't fight
  those protections.
- Entry is spike-free: the jog integration is gated on `s_pos_on`, which the cascade sets only after
  latching `s_pos_hold_rad = actual`, so the first tick has `Δref = 0`.
- Motor-internal only — reuses `0x2200:4`; no OD/contract/`MC_IF_PROTOCOL_VERSION`/CHANGELOG change.
- **Follow-up:** acceleration feedforward (`a_ff`) is still 0 for the jog; a second difference of the
  reference would supply it but is noisy and second-order — left out. Verify tighter tracking on the
  bench (jog at constant speed; the position error should collapse toward 0 vs the old `jog_vel/Kp`).
