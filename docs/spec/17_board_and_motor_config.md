# Board and Motor Configuration

Defines the two foundational runtime-configuration contracts that replace the old
compile-time `motor_config.h` `#define`s, enabling one framework binary to target different
boards and motors. See ADR-004.

## Why

The framework is generic over hardware (multiple power boards) and motors. Hardware-specific
values must therefore be **data**, not preprocessor constants. Two contracts capture this:

- `MC_BoardConfig_t` — everything about the *board* (power stage + sensing + wiring).
- `MC_MotorModel_t` — everything about the *motor* (SI electromechanical model), independent
  of how it is driven.

The proven reference board and its motor are shipped as **profile 0** /
**default motor** for first bring-up; persistence and the object dictionary may override both
at runtime.

## Motor model (`include/mc_motor_model.h`)

Backend-agnostic SI parameters: topology (pole pairs, phase count), electrical model
(R, L, Kt, Ke), mechanical model (inertia, Coulomb + viscous friction), ratings (nominal /
stall current and torque, max speed), and a first-order winding thermal model. FOC turns
torque into `iq` via `Kt` inside the backend; the model itself never references id/iq.

**Default motor — Maxon EC 90 flat (500267):** pole_pairs 11, phase_count 3,
R 0.844 Ω, L 1.07 mH, Kt = Ke 0.231, J 5.06e-4 kg·m², nominal 24 V / 4.06 A / 0.964 Nm,
stall 56.9 A / 9.41 Nm, max speed ≈ 523.6 rad/s (5000 rpm), thermal τ 54.3 s, rise-at-nominal
100 degC, winding max 125 degC. Friction defaults to 0 (tuned/calibrated per axis).

## Board configuration (`include/mc_board_config.h`)

| Group | Fields | Profile 0 (reference board) |
|---|---|---|
| Current sense | shunt, amp gain, zero-current offset V, ADC Vref, ADC full-scale, per-phase sign | 0.01 Ω, 5.18, 1.71 V, 3.3 V, 4096, signs TBD |
| Bus sense | divider ratio, ADC Vref, full-scale | **divider ratio: confirm**, 3.3 V, 4096 |
| Temp sense | analog-present flag, scale, offset | analog_present = false (use thermal model) |
| PWM | frequency, period counts, dead-time, complementary, polarity | 20 kHz, ARR 4250, **dead-time: confirm**, complementary, active-high |
| Encoder (board-level) | backend, counts/rev, bits, invert, mechanical zero | SSI, 2,097,152, 21, invert = true, zero from calibration |
| Handles (opaque) | PWM timer, ADC phase A/C, ADC Vbus, encoder SPI, link SPI | wired at init (TIM1, ADC1/ADC2, SPI1, SPI2) |

Derived helpers: `MC_CurrentSense_AmpsPerCount` (= Vref / (full_scale · gain · shunt) ≈
0.01555 A/count for profile 0) and `MC_CurrentSense_ZeroCount` (≈ 2124 counts).

CubeMX owns peripheral init; opaque `void*` handle fields are bound by the STM32 boundary
modules so this header carries no HAL types and stays host-compilable.

## Values to confirm per board (not present in the old firmware)

The old `sensors.c` does not convert Vbus or temperature ADC channels, so these are genuine
unknowns and must be supplied/measured before they are trusted (process rule #10):

- **DC-bus divider ratio** — no Vbus conversion exists in the reference firmware.
- **Dead-time counts** — confirm from the CubeMX TIM1 configuration.
- **Temperature-sense scaling** — reference uses a model, not a sensor; default to the
  motor thermal model unless an analog sensor is wired.
- **Per-phase current signs** — confirm against phase wiring / determined during calibration.

## Open items

- Board-profile **selection** mechanism: compile-time tag, runtime table, or OD/persisted.
- Whether load inertia is a separate field or folded into one inertia term.

## Verified on hardware (bring-up)

- PWM carrier **20 kHz**, 50 % balanced output observed (motor disconnected); the fast-loop /
  ADC-trigger marker (GPO_1) lands in the low-side conduction window.
- Complementary outputs configured **in phase** with the main channels (`OCPolarity`
  active-high, `OCNPolarity` active-low) — matches the proven board and the gate-driver
  expectation; idle state RESET (low) when disabled.
- **`deadtime_counts = 200`** ⇒ DT = (32+8)×8×t_DTS ≈ **1.88 µs** (DTG register 0xC8 uses the
  ×8 prescaled range, NOT a linear count), confirmed from the TIM1 config (byte-identical to
  bldc_axis_controller). At a 24 V bus this gives a ~1 V modulation deadband — small commanded
  voltages produce little current until past it.
- Encoder sign confirmed: **mechanical position increases for clockwise output rotation**
  (viewed from the output shaft; rotor on the underside). `direction = -1` is correct — this is
  the system-wide positive convention (positive velocity = CW output). FOC torque sign is made
  consistent with it at alignment (phase order + electrical offset).
