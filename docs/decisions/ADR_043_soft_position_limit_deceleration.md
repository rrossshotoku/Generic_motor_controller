# ADR-043: Soft position-limit deceleration + home-gate

- **Status:** Accepted
- **Date:** 2026-06-26
- **Relates to:** ADR-040 (motor motion envelope + soft position limits), ADR-038 (cold-boot position anchor), ADR-042 (velocity-demand smoothing), ADR-028 (D3 position cascade)

## Context

ADR-040 added motor-owned soft position limits (`0x2600:6/7 pos_limit_lo/hi_rad`, home-relative,
manually set). Its first cut: clamp the position-move target into `[lo, hi]` (correct — the planner
glides there) and, in velocity/joystick mode, **hard-zero** the velocity demand at the limit. Two
problems with the hard-zero:

1. **Overshoot.** It reacts only once the actuator is *already* at the limit, so the actuator coasts
   past by its stopping distance (e.g. 5 rad/s at 50 rad/s² ≈ 0.25 rad / ~14° past).
2. **Slam.** Zero-in-one-tick decelerates as hard as the current loop allows — the opposite of the
   ADR-042 smoothing.

Also, the limits are home-relative but were enforced even before the mechanical zero was set, so
`[lo, hi]` was being applied against a meaningless reference.

## Decision

1. **Decelerate to the limit (velocity/joystick mode).** Replace the hard-zero with a distance-based
   velocity taper: cap the demand at `v_allow = sqrt(2 · max_accel · distance_to_limit)`, so the demand
   falls to zero exactly *at* the limit — a smooth glide, no slam, no overshoot. Decel budget = the
   envelope `max_accel` (`0x2600:5`). With `max_accel = 0` (envelope disabled) fall back to the hard
   stop at the limit (ADR-040 behaviour). Position mode is unchanged — its target clamp + the planner
   already decelerate.
2. **Gate on the mechanical zero.** All three enforcement sites (target clamp, velocity taper,
   `AT_LIMIT_LO/HI` bits) are active only when the mechanical zero is set — `s_home_offset_rad != 0`,
   the same signal as `MC_IF_CAL_DONE_MECH_ZERO`. Not zeroed → limits fully off. (They are
   home-relative; without the zero the band is meaningless. The ADR-038 persisted home loads before
   this matters on a cold boot.)
3. **Always allow escape.** The taper/stop restricts only motion *toward* the near limit; motion
   *away* is never limited. So if the actuator is past a limit (overshoot, hand-moved, or a limit set
   while outside the band) you can always drive back in. Position mode escapes by the target being
   clamped into the band.
4. **Torque/current mode stays unguarded** (low-level test/bring-up mode).

## Consequences

- `mc_scheduler.c`: a `pos_limits_active()` helper (`lo < hi && homed`) gates the three sites; the
  velocity guard becomes the taper (`sqrtf`). **No new OD entries, no contract change** —
  `0x2600:6/7` already exist; only the entry *behaviour* is refined, so `MC_IF_PROTOCOL_VERSION` is
  unchanged (CHANGELOG description touched for accuracy).
- GUI: **"capture current position"** buttons set `lo`/`hi` to the live position — the natural way to
  teach limits at the bench.
- Soft limits become usable at speed (glide, not slam), self-recoverable (escape), and inert until the
  axis is zeroed.

## Rejected

- **Keep the hard-zero.** Slams and overshoots; unusable at speed.
- **A dedicated soft-limit decel knob.** `max_accel` already is "how hard may the motor decelerate";
  a second knob is redundant.
- **A motor-side "capture" command** (new OD entry). The GUI writing the live position to `0x2600:6/7`
  needs no contract change.
