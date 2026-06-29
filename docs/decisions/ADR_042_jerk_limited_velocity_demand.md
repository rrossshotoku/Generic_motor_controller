# ADR-042: Velocity-demand acceleration ramp (joystick smoothing)

- **Status:** Accepted
- **Date:** 2026-06-26
- **Relates to:** ADR-021 (joystick is a CMC concept), ADR-040 (motion envelope), ADR-025 (trajectory)

## Context

A joystick — or any live operator velocity input — reaches the motor as a `velocity_setpoint`
(0x60FF) in PROFILE_VELOCITY mode (ADR-021; the motor has no joystick concept). The CMC streams it
in coarse ~25 ms steps, so the velocity-loop reference *steps* 40×/s — felt as a kick / "crunchy"
motion. We want to smooth the **velocity demand** without affecting the position cascade (which
profiles itself, ADR-025) or the tuning generator (which needs the raw reference).

## Decision

A velocity-demand **acceleration ramp** in front of the velocity loop, applied **only to the
PROFILE_VELOCITY demand**; the position cascade and the tuning generator bypass it. The output
velocity is slewed toward the demand under an acceleration cap, and the acceleration itself is
**jerk-limited on the way *up* to the cap but falls *freely* on the way down** (asymmetric).

Per medium-loop tick (dt = 1 ms):

1. `acc_target = clamp((demand − vel)/dt, ±accel_cap)` — the acceleration to land exactly on the
   demand this tick, capped at the phase's max acceleration.
2. Move the applied acceleration toward `acc_target`: its **magnitude may rise by ≤ `jerk·dt`** per
   tick, but may **fall without limit**. (On a sign flip it falls to 0 for free, then the rise is
   jerk-limited in the new direction.)
3. `vel_out = vel + acc·dt`, never crossing the demand.

**Parameters** (OD 0x2300, F32 RW PERSIST, default 0):

- `accel_up` (`:6`), `accel_dn` (`:7`) [rad/s²] — the acceleration cap while speeding up (|v| growing)
  / slowing down. `0` = that phase disabled (pass-through).
- `accel_jerk` (`:8`) [rad/s³], **shared** — how fast the acceleration eases up to the cap. `0` = step
  (the acceleration jumps straight to the cap, i.e. a plain accel ramp). The **down direction has no
  knob** — the acceleration falls freely.

**Why the asymmetry is stable.** The overshoot in the rejected symmetric jerk limiter came from the
acceleration being unable to wind *down* in time at the setpoint — it was at its peak when the
velocity arrived and carried it past. Make the wind-down free and the acceleration can always fall to
track the demand to zero exactly as the velocity lands → **no overshoot**. The jerk-limited *rise*
only shapes the start, so it cannot destabilise. **Feel:** eased starts (no kick when the stick
moves), snappy stops (the acceleration cuts off as it lands). Validated numerically — step, stop, and
+v→−v reversal all show zero overshoot, and `accel_jerk = 0` reproduces the plain accel ramp.

- **Bump-free:** the limiter state (velocity + applied acceleration) resets to the live velocity / zero
  acceleration each tick unless PROFILE_VELOCITY is the active driven mode, so entering velocity mode
  never jumps from a stale state.
- Joystick stays a **CMC concept** — no new motor mode; the filter keys off the velocity-vs-position
  distinction, which is exactly "don't smooth the position path."

## Consequences

- Motor OD entries `0x2300:6 vel_accel_up`, `0x2300:7 vel_accel_dn`, `0x2300:8 vel_accel_jerk`
  (F32 RW PERSIST, default 0). Additive, non-PDO → **no `MC_IF_PROTOCOL_VERSION` bump**.
  CHANGELOG [4.5.0]. GUI: three editable fields + a **Bypass** toggle next to the joystick slider
  (Motor Command), and in the velocity config group.
- **Operator surface (CMC).** These are operator-facing joystick-feel params. They stay motor-owned
  (the smoothing runs on the motor), but the operator configures them *through* the CMC — the OD
  gateway proxies the writes (the PC tuning GUI already does exactly this). The CMC should surface
  them in its operator joystick config UI next to the `axis_manager` calibration
  (`0x3022 joystick_max_velocity`, `0x3027–0x302A` raw-cal/deadband). Caveat: these persist on the
  **motor** while the `axis_manager` calibration persists on the **CMC**, so a single operator
  "save joystick setup" must trigger a save on both sides.
- Applied from the OD in `od_apply_gains`; the filter runs in the medium loop on the
  PROFILE_VELOCITY branch.
- **Default `accel_jerk = 0` ⇒ the acceleration steps to the cap = a plain accel ramp** — i.e. the
  prior behaviour is unchanged until the operator sets a jerk, so this is a pure addition.
- The acceleration cuts off sharply at the setpoint (free fall) — a small jolt at the arrival, *by
  design* ("don't smooth the stop"); negligible for the gradual stick moves this targets.

## Rejected

- **One-sample jerk clamp, symmetric** (the first attempt — built and tested): clamp the acceleration's
  rate of change in *both* directions. **Unstable for a held demand** — the acceleration is an
  *integrated state* that builds up the whole way up the ramp and is at its **peak** exactly when the
  velocity reaches the demand, so it carries the velocity past → overshoot → a slow limit cycle
  (observed on hardware). Making the wind-down free (this ADR) removes it.
- **Jerk-limited on both rise *and* fall** (a full S-curve with braking-distance lookahead): stable
  *and* kick-free at both ends, but materially more code; the asymmetric ramp buys the start-smoothing
  for far less, accepting the sharp stop.
- **A separate "joystick" motor mode**: re-introduces the application concept ADR-021 removed; the
  velocity-vs-position distinction already gates the filter.
- **Smoothing on the CMC**: the motor already owns demand-shaping (the ADR-040 envelope), so it lives
  here and works for any velocity source.
