# ADR-004: Per-board hardware configuration layer + generic motor model

## Status

Accepted

## Date

2026-06-19

## Context

The framework must support varied hardware (option: "new / multiple boards too"), with an
initial profile based on the proven reference board. The old design hard-codes motor,
encoder, current-sense, PWM, and limit values as compile-time `#define`s in
`motor_config.h`. A generic framework needs these as runtime configuration so one binary can
target different boards and motors.

## Decision

Introduce two foundational contracts:

1. **`MC_BoardConfig_t`** (`include/mc_board_config.h`) — per-board hardware: current-sense
   scaling, DC-bus and temperature sense scaling, PWM timing, board-level encoder facts, and
   opaque CubeMX handle bindings wired by the STM32 boundary modules. No HAL types, so it is
   host-compilable.
2. **`MC_MotorModel_t`** (`include/mc_motor_model.h`) — motor-type-agnostic SI
   electromechanical model (pole pairs, R, L, Kt, Ke, inertia, friction, ratings, thermal
   model).

Ship **board profile 0** (`MC_BoardConfig_LoadProfile0`) and the **default motor**
(`MC_MotorModel_LoadDefault`, Maxon EC 90 flat 500267) from the proven reference board for
first bring-up. Persistence/OD may override both at runtime.

## Reasoning

Replacing `#define`s with config structs is the central enabler of the "generic" goal and of
the future brushed-DC backend. Keeping FOC interpretation (torque→iq via Kt) in the backend
keeps the model itself backend-agnostic. Opaque handle pointers honour "CubeMX owns
peripheral init" while letting boundary modules bind handles at startup.

## Reasoning — values

Current-sense and PWM values for profile 0 are known-good from the old `motor_config.h`
(shunt 0.01 Ω, gain 5.18, offset 1.71 V, 12-bit @ 3.3 V; PWM 20 kHz, ARR 4250). Motor default
is Maxon 500267 (11 pole pairs, R 0.844 Ω, L 1.07 mH, Kt=Ke 0.231, J 506 µkg·m²).

## Consequences

- New public headers `mc_board_config.h`, `mc_motor_model.h`; new spec
  `docs/spec/17_board_and_motor_config.md`.
- Boundary modules (`mc_current_sense`, `mc_pwm`, `mc_ssi_encoder`) consume board config
  instead of `#define`s; FOC/current-request consume the motor model.
- Per-board profile selection mechanism needed (compile-time tag now; OD/persisted later).

## Files affected

- include/mc_board_config.h
- include/mc_motor_model.h
- docs/spec/17_board_and_motor_config.md
- requirements.yaml

## Open questions

- **Hardware values not in the old code (confirm per board before relying on them):**
  DC-bus divider ratio, dead-time counts, temperature-sense scaling, per-phase current signs.
  Old firmware does not convert Vbus/temperature ADC at all; temperature defaults to the
  motor thermal model.
- Profile selection: compile-time `#define`, runtime table, or persisted/OD-selected?
- Does the motor model carry load inertia separately, or fold it into one inertia term?
