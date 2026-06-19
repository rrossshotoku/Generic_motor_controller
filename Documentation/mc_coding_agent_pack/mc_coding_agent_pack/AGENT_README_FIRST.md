# AI Coding-Agent Brief

You are implementing an STM32G474RET3 motor-controller framework in C.

## Hard rules

- Use C, not C++.
- Do not modify CubeMX-generated code except inside `USER CODE` sections or in explicitly user-approved wrapper files.
- Keep framework code in `mc_*` modules.
- Keep internal units in SI: rad, rad/s, rad/s^2, rad/s^3, Nm, A, V, degC.
- Do not put raw ADC/SPI/TIM register details inside high-level controllers.
- Do not let position/velocity controllers read object dictionary entries directly.
- Do not let FOC know about joystick, profile position, mode manager, or network protocol.
- Do not allocate memory dynamically in real-time paths.
- Do not block in the 20 kHz fast loop.
- Do not write flash from fast or medium loops.
- Preserve the ability to add a future brushed DC backend.

## Implementation style

- Implement one module at a time.
- Keep public APIs stable once downstream modules use them.
- Add small host-buildable tests where practical.
- Compile after each stage.
- Use `float` for control logic unless a module explicitly says fixed-point or CORDIC may be used.
- HAL/CubeMX owns peripheral setup. LL/direct-register access is allowed only in timing-critical STM32 boundary modules.

## Ask before choosing these hardware-specific details

The user will configure CubeMX. Do not invent final values for:

- Exact ADC channel assignments
- Exact SPI instance for SSI or inter-MCU link
- Exact TIM1/TIM8 choice
- Exact GPIOs, DMA streams, and interrupt priorities
- Exact current-sense gain and offset values
- Exact encoder bit layout for a specific encoder model

Use configuration structs and wrapper functions so the user can wire the CubeMX handles later.

## First implementation objective

Create a compileable framework skeleton with clear module boundaries, then implement modules incrementally in this order:

1. Shared types, status codes, units, limits
2. Reusable PID/PI block
3. Object dictionary static table and typed read/write API
4. SPI frame encode/decode and CRC16
5. State estimator and SSI encoder backend interface
6. Current-sense and PWM STM32 boundary stubs
7. FOC basic math and current PI loops
8. Current/torque request generator
9. Position and velocity controllers
10. Mode manager and command paths
11. Jerk-limited trajectory planner
12. Fault manager
13. Calibration manager
14. Persistent parameter store
15. Scheduler integration hooks
