# ADR-039: Brushed-DC / H-bridge drive backend (board profile 1) — same board, two legs as an H-bridge

- **Status:** Accepted (approach + hardware mapping); implementation staged on branch `brushed-dc-backend`.
  Sub-decisions marked **OPEN** below settle per-stage (some as their own ADRs).
- **Date:** 2026-06-25
- **Related:** ADR-001 (scope: motor MCU; "future brushed-DC/H-bridge backend must remain possible"),
  ADR-004 (per-board config + `MC_MotorModel_t` + `backend_type`), ADR-006 (scheduling/fast loop),
  ADR-007 (current sense), ADR-009 (PWM backend), ADR-011 (FOC current loop), ADR-012 (torque→current seam),
  ADR-037/038 (single-turn startup anchor — superseded for the quadrature path).

## Context

The framework was built motor-type-agnostic and the brushed backend was reserved up front:
`MC_MOTOR_BACKEND_BRUSHED_DC_HBRIDGE = 1` (`mc_types.h:97`), `MC_MotorModel_t` carries `backend_type`,
`pole_pairs` ("1 for brushed DC"), `phase_count` ("1 for brushed DC"), the position-sensor sample has an
`absolute` flag "for SSI and future quadrature backends", and the calibration enum already lists
`MC_CAL_HOMING` + `MC_CAL_SOFT_LIMITS`. The outer motion stack (trajectory → position → velocity →
torque/current request) is agnostic; the seam is **the current command in amps**. FOC concepts
(Park/Clarke/SVPWM, electrical angle, id/iq) are confined to the FOC backend.

We now adapt for a real brushed-DC actuator (quadrature encoder, hard end-stops). The drive electronics
are the **same board / same `.ioc`**: the 3-phase inverter's legs A and B form the H-bridge across the
motor terminals; leg C is unused. The half-bridge drive circuit is identical to the BLDC, so current
scaling is unchanged. (Confirmed against `adc.c`, `mc_current_sense_stm32g474.c`, `tim.c`.)

This ADR covers the **drive backend + current sensing + backend selection**. The **quadrature encoder /
estimator** change and **bottom-stroke homing** are deliberately separate follow-ups (the encoder needs a
`.ioc` change the drive does not).

## Decision

**1. Board profile 1 (brushed).** `backend_type = MC_MOTOR_BACKEND_BRUSHED_DC_HBRIDGE`, `phase_count = 1`,
`pole_pairs = 1`. H-bridge = **leg A (PWM_PHA/nPWM_PHA, TIM1 CH1/CH1N) + leg B (PWM_PHB/nPWM_PHB, CH2/CH2N)**;
**leg C (CH3/CH3N) forced safe-off**. The `.ioc` is reused unchanged for the drive: each leg is already a
complementary pair with dead-time + MOE — exactly an H-bridge leg.

**2. Current sensing — both driven legs, one armature current.** Repoint **ADC2 from IN6 (`I_C`, PC0) to
IN7 (`I_B`, PC1)** — the pin is already analog-configured (`adc.c:226`); only the regular channel changes
(`adc.c:132`, `ADC_CHANNEL_6`→`ADC_CHANNEL_7`), reflected in CubeMX so the `.ioc` stays canonical. The dual
**regular-simultaneous** capture then samples `I_A` (ADC1) and `I_B` (ADC2) at the *same instant* (the sync
is ADC1↔ADC2, independent of channel — see ADR-007). The motor is a series element across the bridge, so
there is **one** armature current measured at both ends (`I_A ≈ −I_B`):

```
i_armature = (I_A − I_B) / 2        // both ends; common-mode + offset rejection
health:      |I_A + I_B| ≈ 0        // a non-zero sum flags a shunt/amp/wiring fault on one leg
```

Drop the 3-phase Kirchhoff reconstruction `ib = -(ia+ic)` (`mc_current_sense_stm32g474.c:98`) — it
synthesises a phase we don't measure; here both driven legs are measured directly.

**Hardware note — new board pinout (found during bring-up, 2026-06-25).** The brushed board reassigns the
current-sense pins vs the reference board: **`I_A` → PC1 (ADC2_IN7)**, **`I_B` → PA0 (ADC1_IN1)**, and the
old **`I_C` (PC0 / ADC2_IN6) is unused/floating**. So the firmware default (ADC2 = IN6) reads the floating
ex-`I_C`, the reconstruction `ib = -(ia+ic)` rails, and the over-current monitor sees **~65 A at idle** and
false-trips. Immediate fix: the brushed over-current trip uses the armature leg on **ADC1 only** (`|ia|`),
and the current loop reads `i_arm = ia_a` (= `I_B` on this board — a valid single-leg armature current).
The ADC2 IN6→IN7 repoint then reads `I_A` on PC1, giving the dual-leg `(I_A − I_B)/2`. **Action:** give the
new board its own `.ioc`/board profile and **audit the full pinout** (PWM legs PHA/PHB/PHC, Vbus, temp,
enables) for other reassignments before driving.

**Backend selection + bring-up tooling (2026-06-25).** The backend is chosen by a persisted OD entry
**`0x2000:6 motor_backend_sel`** (0 = BLDC/FOC, 1 = brushed), read **once at boot** to set
`s_motor.backend_type` *and* the current-sense ADC channel (FOC = ADC2_IN6; brushed repoints ADC2 to IN7 =
the new board's `I_A`). So it is per-board config and applies on **save + reboot** — the GUI shows a
BLDC/Brushed dropdown in Motor Config that writes it. (The earlier unconditional `SelectHBridgeLegs()` is
now gated by this; a brushed board must have it set/saved or it boots FOC on IN6 → the floating-`I_C`
trip.) Contract: CHANGELOG [4.5.0], additive, no version bump. Scaffolding added alongside: a debug DAC
(PA4) streaming `i_max_a` (`mc_dac`); the armature-current PI tuned from R/L (`kp=ωc·L, ki=ωc·R`); an
open-loop voltage mode for the current-sign check; and a **current tuning mode** (`test_mode=3`, extends
ADR-030) that drives the current command from the signal generator (amplitude in A, rate 0 = step pulse). The
current-loop gains moved to the OD (`0x2400:6/7 hb_cur_kp/ki`, PERSIST, GUI-settable on Motor Config —
replacing the watch-window seed); the measured armature current is published (`0x2410:6 tlm_i_arm_a`)
for a live Motor-Config readout + graphing.

**3. Backend dispatch at the current→PWM seam.** Today the fast loop hard-calls `MC_Foc_Update` →
`MC_Pwm_SetDutyFast` (`mc_scheduler.c`, MC_FastLoop_20kHz). Introduce a selection on
`MC_MotorModel.backend_type` at that seam: FOC path unchanged (3-phase SVPWM duty); new brushed path =
single armature-current loop → H-bridge modulator → 2-leg duty (leg C off). **No electrical angle, no
commutation, no Park/Clarke.** Whether the seam is a function-pointer backend or a `switch` on `backend_type`
is an implementation detail (**OPEN-A**); the seam *location* (current command in → PWM duty out) is fixed.

**4. Outer layers reused unchanged.** Trajectory, position/velocity controllers, `mc_pid`,
`mc_current_request` (torque→current via Kt is identical for brushed), mode manager, comms/OD/persistence,
`mc_signal_gen` tuning. The public command stays torque/current — no behaviour change above the backend.

**5. Over-current trip.** Reuse the existing trip but on `|i_armature|`. (With the present sign convention
`max(|ia|,|ib|,|ic|)` already resolves to `|i_armature|`, but make it explicit for the brushed path.)

**6. Contract.** Board profile + backend are motor-internal → **no shared-contract change, no
`MC_IF_PROTOCOL_VERSION` bump**. FOC-specific OD entries (id/iq gains `0x2400`, electrical offset
`0x2500:1`, `pole_pairs`) become **N/A** for brushed — leave them inert, and backend-gate their display in
the GUI / calibration-completeness so they aren't shown as outstanding.

### OPEN sub-decisions (settle per-stage; forking ones get their own ADR)

- **RESOLVED-A — backend seam:** in-place branch on `backend_type` at the fast-loop current->PWM seam
  (`mc_scheduler.c` MC_FastLoop_20kHz). FOC path kept byte-for-byte (its `if` became `else if`); brushed branch
  added as a safe-off stub until OPEN-B/C. Backend *logic* lives in modules (`mc_foc`, future `mc_hbridge`),
  not a vtable — keeps the 20 kHz path flat and the refactor zero-risk. `g_mc_debug.backend_type` mirrors the
  active backend for the watch window.
- **RESOLVED-B — modulation = locked anti-phase.** Both legs PWM in anti-phase about 50% (50% = 0 V,
  `V_motor = Vbus·(2·duty_a − 1)`, `duty_b = 1 − duty_a`) → linear through zero, no zero-crossing dead-band.
  Chosen over sign-magnitude for a **park-on-target** actuator: the configured **~1.9 µs dead-time**
  (`DeadTime=200`, decoded at the 170 MHz center-aligned TIM1 clock = `320·t_DTS`, ~3.8 % of the 50 µs period)
  would widen sign-magnitude's near-zero dead-band (low-duty pulse-swallowing + DCM) and risk hunting at the
  setpoint, whereas locked anti-phase runs at 50 % duty (clear of the dead-time rails) and its ripple current
  dithers through the dead-time nonlinearity. Cost accepted: continuous full-bus ripple + switching loss + a
  current-sign-dependent dead-time voltage offset (compensable, ~1–2 V at 24 V). Modulator = new `mc_hbridge`
  (`MC_HBridge_LockedAntiphase`: signed armature voltage → leg-A/B duties, leg C off). Later levers (not now):
  trim the dead-time toward the FET/gate-drive limit, and dead-time feed-forward by current sign.
- **RESOLVED-C — current loop.** A single armature-current PI (`mc_pid`, fast loop) → voltage command →
  locked anti-phase modulator. Chosen over voltage-mode: accurate torque (`i·Kt`) despite R(temp) drift,
  current limiting from the PI clamp, parity with the FOC tuning workflow, and it actually uses the current
  sensing. First cut: `i_arm = I_A` (single-ended, valid); dual-leg `(I_A−I_B)/2` follows with the ADC2→IN7
  reconfig. Backend flipped live for bring-up via `inject_enable + brushed_backend` (ADR-005, no rebuild);
  gains `hb_kp`/`hb_ki` live-tunable; `fw_build` bumped to 39. Module: `mc_hbridge`; loop in
  `mc_scheduler.c` MC_FastLoop_20kHz brushed branch.
- **OPEN-D — quadrature encoder + estimator:** incremental (no single-turn seam, no electrical angle); needs
  a `.ioc` change (timer encoder-interface mode, not SPI1-SSI). **Separate ADR**; the ADR-037/038 single-turn
  anchor does not apply (homing sets the zero).
- **OPEN-E — bottom-stroke homing + soft limits:** `MC_CAL_HOMING` (already enumerated) drives to the end-stop
  (current/stall detect), captures encoder zero; activates the reserved `AT_LIMIT_LO/HI` `movement_status`
  bits (ADR-033) + `MC_CAL_SOFT_LIMITS`. **Separate ADR.**

## Consequences

- One PCB / one `.ioc` serves both motor types; selection by board profile. Minimal new firmware (H-bridge
  modulator + current combine + backend dispatch); the bulk — outer cascade, comms, OD, persistence, tuning —
  is reused as-is, validating the ADR-001/004 agnostic design.
- Leg C **must** be guaranteed safe-off on the brushed path (the FOC path drives all three).
- **Confirm per board** (CLAUDE.md hardware list): `phase_current_signs` for the new wiring (positive
  `i_armature` = intended "forward"), and that the low-side-shunt sample window at the PWM peak still captures
  the armature current in the H-bridge freewheel.
- Encoder + homing are staged; until the quadrature work lands, an SSI build still runs on profile 0.

## Files (planned this branch)

`include/mc_board_config.h` + `src/mc_board_config.c` (profile 1), new `mc_drive_backend` / `mc_hbridge`
module, `src/mc_pwm*` (H-bridge duty), `src/mc_current_sense_stm32g474.c` (ADC2→IN7, `(I_A−I_B)/2`, health
check), `src/mc_scheduler.c` (backend dispatch in the fast loop), `Core/Src/adc.c` + `.ioc` (ADC2 channel).
Docs: this ADR, `ADR_000_decision_log.md`, `requirements.yaml`, `docs/spec/17_board_and_motor_config.md`,
`docs/spec/11_feedback_and_encoders.md` (encoder follow-up). Separate ADRs for OPEN-B/D/E.
