# ADR-046: GUI-fireable open-loop d-axis voltage pulse for plant identification

- Status: Accepted
- Date: 2026-06-28
- Related: ADR-024 (electrical alignment / open-loop Vd), ADR-018 (command-source arbitration),
  ADR-005 (bring-up over SWD)

## Context

Identifying the electrical plant (winding R and L) needs a known voltage applied to the
d-axis at standstill: d-axis current produces no torque on a non-salient PMSM, so once the
rotor is aligned it stays put while `i_d` responds first-order (`i_d(t) = (V/R)(1 - e^{-tR/L})`).

The motor already has an open-loop d-axis voltage path — the electrical-alignment routine
(ADR-024) drives `Vd` at a forced electrical angle through the commissioning/inject
arbitration (ADR-018). Until now that path was reachable only from the watch-window inject
struct (`g_mc_inject`, over SWD). The operator wants to fire the step from the PC tool.

## Decision

Expose the open-loop d-axis voltage step over the OD as a motor-owned commissioning **pulse**:

- `0x2900:6 dq_test_voltage_v` (F32) — open-loop `Vd` [V].
- `0x2900:7 dq_test_angle_rad` (F32) — forced electrical angle [rad] (`0` = d-axis on phase A,
  so `i_d = i_a`).
- `0x2900:8 dq_test_enable` (U8) — write `1` to fire.
- `0x2900:9 dq_test_dwell_ms` (U16) — hold time before auto-return to 0.

The command arbitration (`mc_scheduler.c`) gains a third branch, priority
**watch-inject > OD dq-test > remote**: when `dq_test_enable` is set (and the watch-window
inject is not), the scheduler drives the existing align path with the OD `Vd`/angle for
`dq_test_dwell_ms`, then **returns `Vd` to 0 and disarms** (`dq_test_enable -> 0`). `Vd` is
clamped to ±`MC_C2_VD_MAX` (**12 V** ≈ Vbus/2, the SVPWM ceiling — raised from 3 V so plant ID
can use higher voltages where the ~0.9 V dead-time offset is a smaller fraction). This is the
**shared** open-loop clamp, so the alignment / manual-commissioning ceiling rises too — accepted:
the electrical-alignment routine current-regulates to its target (its current is unchanged), the
PWM duty saturates beyond ~Vbus/2, and the over-current trip bounds current. The dwell is clamped
to `MC_DQ_TEST_MAX_MS` (10 s) as a backstop.

This is a **motor-level** operation — it does NOT go through the CMC axis_manager. The GUI
writes the OD entries; the CMC's OD bridge forwards them like any motor entry. Because the
commissioning path overrides the remote enable, the motor energizes regardless of the
axis_manager mode (and reports `ENABLED` in its statusword) — set axis_manager OFF so the CMC
isn't also commanding the motor.

GUI: a "Plant ID — d-axis voltage pulse" section (Motor Config) with `Vd` / angle / dwell
fields, **Fire pulse** (confirmation; auto-points the debug DAC at `ia` so PA4 shows `i_d`)
and **Stop** (abort early).

## Dead-time caveat (why single-step, "option B")

The bridge has ~1.9 µs dead time (`TIM1` DTG=200 @ 170 MHz) and **no dead-time compensation**,
so the applied voltage ≈ `V_cmd - ~0.9 V·sign(i)` at `Vbus = 24 V`. Therefore:

- A single-point `R = V / i_d,ss` is biased high (~2× at 1.5 V) and is **not** trustworthy.
- **L** (from the transient time constant `τ = L/R`) is robust to the ~constant offset.
- **R** should be taken from a **V–I sweep**: fire several `Vd`, plot `i_d,ss` vs `V`,
  `R = 1/slope` (dead time cancels), `x`-intercept ≈ `V_dt`.

The GUI ships the single-pulse primitive (operator sweeps `V` manually and fits the slope off
the scope); an automated sweep is a possible future enhancement.

## Consequences

- New motor-owned OD entries (additive, non-PDO, not persisted) — no `MC_IF_PROTOCOL_VERSION`
  bump; CMC unaffected (transport). Logged in `Interface/CHANGELOG.md`.
- Reuses the proven align path; `MC_C2_VD_MAX` raised 3 → 12 V (≈ Vbus/2). This is the **shared**
  open-loop d-axis clamp, so the alignment + manual-commissioning ceiling rises too — accepted:
  the alignment current-regulates to its target, so its current is unchanged, and the OC trip
  remains the current safety for all open-loop paths.
- The operator must treat "dq_test armed" as "motor live regardless of axis_manager."
- The dead time itself (~1.9 µs, large) is worth revisiting per-board — halving it halves the
  voltage distortion.
