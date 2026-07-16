# ADR-065: Motor thermal model (I²t) with progressive derate

- Status: Accepted
- Date: 2026-07-09
- Related: ADR-058 (fault system / 0x2600), ADR-042 (velocity current limit path), ADR-004 (per-board / motor model), ADR-019 (OD generated from the shared contract)

## Context

Motors have a thermal limit the drive should respect, but operators specify it in
different ways depending on the datasheet: sometimes a **continuous current rating**,
sometimes an **intermittent duty rating** ("max current for 2 min, then 18 min off").
We want a single protection that works from whichever spec is available, without a
temperature sensor requirement (the reference board's temp-sense scaling is not
calibrated per board), and that keeps the machine running rather than tripping on the
first transient.

## Decision

**One first-order (I²t) thermal model, parameterised by exactly two numbers.** Treating
the winding as a single lumped thermal mass and working in *utilisation* `x = θ/θ_max`
(0 = cold, 1 = at the limit) rather than absolute °C, the entire model collapses to:

```
    dx/dt = (1/τ_th) · [ (I / I_cont)² − x ]
```

Every physical constant (winding R, thermal resistance, thermal capacitance, θ_max)
either cancels (we measure fraction-of-limit, not °C — which is *why* it needs no sensor)
or folds into the two survivors: `I_cont` (steady-state limit) and `τ_th` (time constant).
So the model is exactly two parameters; that is not a simplification of a richer model —
for a single thermal mass it is the complete description.

Chosen behaviours (confirmed with the user):
- **Model-only** (current-based estimator); the temperature sensor is not used at runtime.
- **Progressive derate**: current limit multiplier `= 1` for `x ≤ 0.85`, linearly to `0`
  at `x = 1`; **OVERTEMP fault** only as a backstop at `x ≥ 1.05` (with clear at `x < 1.0`)
  — i.e. if derate cannot hold the temperature (e.g. a stall pinning current).
- **The controller stores only `(I_cont, τ_th)`** and runs one model that has never heard
  of duty cycles. The continuous↔duty distinction lives entirely in a **GUI calculator**
  (Tools tab) that converts a spec into `(I_cont, τ_th)` and writes them.

### Deriving `(I_cont, τ_th)` from either spec (done in the GUI)
- **Continuous:** `I_cont` = the rating directly; `τ_th` from datasheet, a step-test, or a
  conservative default. `I_cont` is the safety-critical number and is correct regardless of
  `τ_th`; `τ_th` only sets how long a burst is tolerated (a small default is safe).
- **Duty** `(I_max, t_on, t_off)`: a single duty point is one constraint on two unknowns, so
  we close it with the physically-grounded assumption that the mandated off-time ≈ full
  thermal recovery → `τ_th ≈ t_off / N` (N ≈ 4, "recovers to ~2%"). Then solving the
  periodic-steady-state cycle for the current that peaks at the limit gives `I_cont`. Worked
  example (2 min on / 18 min off): `τ_th ≈ 4.5 min`, `I_cont ≈ 0.60·I_max` — the model then
  permits the full `I_max` burst and enforces the cooldown, ~double the RMS-equivalent
  (`0.32·I_max`). The one assumption is on the safe side: if the real motor cools faster it
  just derates earlier.

## OD (shared contract, additive — no `MC_IF_PROTOCOL_VERSION` bump; non-PDO)

New block **`0x2100` — motor thermal model** (F32 SI, motor-owned, PERSIST):
`:1 thermal_enable` (U8), `:2 thermal_i_cont_a` (F32, A), `:3 thermal_tau_s` (F32, s),
`:4 thermal_utilisation` (F32 RO, 0–1, PDO-mappable), `:5 thermal_derate_factor` (F32 RO, 0–1, PDO-mappable — both telemetry-mappable into 0x2A00 so the utilisation can be graphed climbing to 1).
`:6 thermal_derate_start` (F32 RW PERSIST, default 0.85, clamped [0, 0.99]) — the utilisation
at which the derate begins. Fault block: **`MC_IF_FAULT_OVERTEMP = 0x8`** in `fault_flags`,
count at `0x2600:14` `fault_count_overtemp`. The derate-END (1.0) and the fault thresholds
(1.05 set / 1.0 clear) stay firmware constants.

## Runtime placement
Slow loop (100 Hz, `dt = 0.01 s`) — thermal constants are seconds-to-minutes so there is no
fast-loop cost. Current input is backend-aware (`√(id²+iq²)` for FOC, `|i_arm|` for brushed).
The derate scales the operational current limit in `od_apply_gains`; the hard OC trip
(`0x2600:2`) is **not** derated (it stays the absolute backstop).

## Consequences
- One control-loop model + one fault path to validate; the "two types" is a GUI concern only.
- Steady-state protection is correct from `I_cont` alone; `τ_th` is a burst-tolerance tuning
  knob with a safe default (0 = "no burst, limit to `I_cont`").
- Disabled by default (`thermal_enable = 0` → derate = 1, no fault) → zero behaviour change
  until an operator configures a motor.
- **Follow-ups:** optional temp-sensor fusion; a step-test `τ_th` characterisation helper;
  promoting `derate_start` to the OD if per-motor tuning is wanted. A golden-reference check
  (constant-overload → `x` trajectory + derate onset vs the closed-form solution) is the
  validation plan; host unit test optional (ADR-002 policy).
