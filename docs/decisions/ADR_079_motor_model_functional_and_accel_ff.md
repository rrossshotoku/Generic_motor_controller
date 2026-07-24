# ADR-079: Make the motor-model parameters drive the control + velocity-loop acceleration feedforward gain

- Status: Accepted
- Date: 2026-07-24
- Related: ADR-012 (torque/current request), ADR-028/031 (position cascade + velocity FF gain),
  ADR-039/049 (direct current-loop gains, R/L promotion), ADR-052 (quad encoder)

## Context

Audit found four of the five `0x2000` motor-model parameters were **not actually used** by the
control:

- `motor_kt_nm_per_a` (0x2000:1) — **used** (torque→iq in `mc_current_request`).
- `motor_inertia_kg_m2` (0x2000:2) — the OD value was **never read**; the acceleration feedforward
  used the built-in model constant `s_motor.rotor_inertia`, not the OD.
- `motor_pole_pairs` (0x2000:5) — the OD value was **never read**; FOC commutation used the built-in
  default. It "worked" only because the default (11) matched the reference motor.
- `motor_resistance_ohm` / `motor_inductance_h` (0x2000:3/4) — reach `s_motor` but no control law
  consumes them (gains are the direct `0x2400:*` entries since ADR-049).

Separately, the position loop has a `velocity_ff_gain` (0x2200:4) to trim its velocity feedforward,
but the velocity loop had **no equivalent** knob for its acceleration feedforward.

## Decision

1. **Wire `motor_inertia` and `motor_pole_pairs` into the control** (`od_apply_gains`):
   `s_torque_cfg.inertia_kg_m2 = g_od.motor_inertia_kg_m2` (→ accel FF) and
   `s_est_cfg.pole_pairs = g_od.motor_pole_pairs` (→ FOC electrical angle). Defaults are unchanged
   (0.000506, 11), so the reference board sees no behaviour change; other motors now configure
   correctly from the OD.
2. **Add `accel_ff_gain` (0x2300:15, F32 RW PERSIST, default 1.0)** — the velocity-loop analog of
   `velocity_ff_gain`. `mc_current_request` now computes `t_accel = accel_ff_gain · inertia · accel_ff`.
   1.0 = full physical FF, 0 = none; a trim on top of the physical inertia. Active on
   `PROFILE_POSITION` moves (jogs have `accel_ff = 0`).
3. **Give `R`/`L` a purpose via a GUI "Estimate from Model" button** on the Current-loop group. For a
   target bandwidth `f`, pole-zero cancellation gives `kp = 2π·f·L`, `ki = 2π·f·R` (crossover ≈ kp/L,
   ki/kp = R/L). It writes the active backend's gains (FOC `0x2400:1–4`, brushed `0x2400:6/7`) as a
   first cut. R/L still drive no control law directly — the loop uses the direct gains — but they now
   *seed* them, so they stop being dead config.

## Consequences

- Inertia and pole_pairs are now honoured — important the moment a different motor is used (wrong
  pole_pairs = wrong commutation; the previous silent default was a latent trap).
- The accel FF is now tunable end-to-end (set the real inertia, trim with `accel_ff_gain`) — better
  following on jerk-limited position moves without over-relying on the position P term.
- The "Estimate from Model" button gives a safe first current-loop tune from the datasheet R/L
  instead of hand-guessing — then verified on the bench.
- Additive OD entry (`0x2300:15`) → no `MC_IF_PROTOCOL_VERSION` bump; PERSIST blob additive-safe (old
  configs restore `accel_ff_gain` at the 1.0 default). CHANGELOG v5.14.0.
- No regression on the reference board (defaults unchanged). **On any other motor, set the correct
  inertia / pole_pairs** — they now matter.
- Verification build + review only. Bench: confirm pole_pairs takes effect (commutation), set a real
  inertia + a jerk-limited move and watch the accel-FF torque term; use the button for a current-loop
  first cut and verify the step response.
