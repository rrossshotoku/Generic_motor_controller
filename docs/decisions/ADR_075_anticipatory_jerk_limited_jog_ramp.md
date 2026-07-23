# ADR-075: Anticipatory jerk-limited (S-curve) velocity-demand ramp for the jog

- Status: Accepted (opt-in; default off)
- Date: 2026-07-23
- Related: ADR-042 (velocity-demand accel ramp), ADR-045 (S-curve trajectory planner), ADR-062
  (position-integrated jog), ADR-074 (stop-integrator bleed)

## Context

The joystick jog slews the velocity demand through `vel_slew_limit()` (ADR-042). That ramp is
jerk-limited **asymmetrically**: the acceleration magnitude may *rise* by ≤ `jerk·dt` per tick, but
is allowed to *fall freely*. The free fall is deliberate — it prevents overshoot of the stick's
velocity setpoint (a naive symmetric jerk limit can't stop the acceleration in time, so the velocity
sails past the target and the loop hunts). But it means the **acceleration is discontinuous** at the
setpoint: as the velocity arrives, the applied acceleration drops from ~`accel_up` to 0 in one tick —
a jerk spike every time the stick settles.

The correct way to jerk-limit a *live* setpoint without overshoot is to start ramping the
acceleration **down early**, timed so it reaches zero exactly as the velocity reaches the target.

## Decision

Add an opt-in **anticipatory jerk-limited S-curve** path to `vel_slew_limit`, selected by
`vel_accel_scurve` (0x2300:14, U8 RW PERSIST, **0 = off** default = the ADR-042 free-fall behaviour).

Each tick, with remaining velocity `e = target − v`, jerk cap `j = vel_accel_jerk`, accel cap
`alim` (`accel_up`/`accel_dn`):

```
a_stop = sqrt(2·j·|e|)              // accel from which we can still ramp to 0 within |e|
a_tgt  = sign(e) · min(a_stop, alim)
a     += clamp(a_tgt − a, ±j·dt)   // ramp the applied accel toward the target at the jerk limit
v     += a·dt                      // (backstop clamps v so it can never cross the target)
```

Holding the acceleration on the phase-plane braking boundary `a = √(2j|e|)` rounds **both** ends of
the ramp: `a → 0` as `e → 0`, so the velocity eases onto the setpoint with **continuous
acceleration** and **no overshoot**. It re-plans against the live target every tick, so a moving
stick just re-tracks (jerk-bounded), and a reversal decelerates through zero and accelerates the
other way, all within the jerk limit — no hunting, because it never overshoots the *current* target.

### Choices

- **Opt-in, default off.** The current free-fall ramp is known-good; this is A/B-able against it by a
  single flag, and the flag lets the operator revert instantly if the S-curve is mis-tuned.
- **Reuses `vel_accel_jerk` (0x2300:8)** as the jerk cap — no new tuning knob; the flag just upgrades
  that jerk from "rise-only" to "both-ends, anticipatory."
- **Backstop retained** (`v` may never cross the target) as a hard no-overshoot guarantee independent
  of the S-curve math — so even a tuning error can't cause overshoot, only a small accel kink.
- Same math family as the `mc_traj_scurve` planner (ADR-045), but applied *incrementally to the
  streaming velocity command* rather than a planned rest-to-rest move.

## Consequences

- Continuous acceleration through the setpoint → no settle jerk; smoother on-camera jog feel, in both
  directions and on reversals.
- Costs a little easing near the target (the rounded approach is slightly gentler than the current
  snap) — the intended trade for smoothness; imperceptible as lag at sane jerk values.
- GUI: the 0x2300 jog-feel knobs (jog_position_mode, accel ramp, this flag, and the ADR-074
  stop-bleed) are regrouped into a new **"Jog parameters"** section; pure loop gains (kp/ki/kd/limit/
  load) stay in "Velocity loop gains."
- Additive non-PDO OD → no `MC_IF_PROTOCOL_VERSION` bump; CHANGELOG v5.11.0.
- **Verification is build + review only** — no hardware here. The S-curve is finicky discrete control;
  bench-validate: enable, set a sane `vel_accel_jerk`, jog and release, confirm the settle jerk is
  gone with no overshoot/hunt; check a hard stick reversal stays bounded. Watch for a limit-cycle at
  the very end (the `√` boundary has infinite slope at 0) — the backstop should absorb it, but if a
  residual chatter shows, add a small "within ε of target → snap `v`, zero `a`" guard.
- Note: `vel_slew_limit` is also used by the homing approach ramp; the flag smooths that too (benign).
