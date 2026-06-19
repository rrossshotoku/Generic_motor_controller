# Implementation Task Sequence

Use this as the coding order. Each task should leave the project compileable.

## Task 1: Common types and build integration

- Add `include/` to the project include path.
- Add `src/` framework files to the build.
- Implement `mc_types.h`, `mc_config.h`, and placeholder module init functions.
- Confirm all files compile as C.

## Task 2: PID/PI primitive

- Implement `MC_Pid_Init`, `MC_Pid_Reset`, and `MC_Pid_Update`.
- Support P, PI, PID modes through config flags.
- Include derivative-on-measurement option and derivative low-pass filtering.
- Include integrator clamping and output limiting.
- Add simple tests for P, PI saturation, reset, and derivative filter behaviour.

## Task 3: Object dictionary

- Implement static object table lookup by index/subindex.
- Implement typed read/write APIs.
- Enforce type, access, range, and size checks.
- Add read callbacks for live values and write callbacks for side effects.
- Include a practical initial OD object set from `docs/spec/05_object_dictionary.md`.

## Task 4: SPI framed protocol

- Implement CRC16.
- Implement frame encode/decode.
- Validate sync, version, length, header CRC, payload CRC.
- Add message types for cyclic process data, OD read/write, heartbeat, and error response.
- Add timeout/sequence tracking interface but leave actual ISR/DMA integration to STM32 wrapper.

## Task 5: Feedback path

- Implement configurable SSI absolute encoder backend interface.
- Implement common position sensor sample and state estimator.
- State estimator outputs mechanical state and electrical angle for BLDC FOC.
- Keep quadrature backend as future extension via same interface.

## Task 6: FOC backend

- Implement basic FOC in C: Clarke, Park, d/q PI loops, voltage limiting, inverse Park, SVPWM.
- Use two PID primitive instances configured as PI loops.
- Keep CORDIC optional behind wrapper functions.
- Keep TIM/ADC details outside FOC.

## Task 7: Motion controllers

- Implement current/torque request generator.
- Implement position controller wrapper using PID block.
- Implement velocity controller wrapper using PID block.
- Implement joystick/profile velocity conditioners.

## Task 8: Trajectory planner

- Implement first jerk-limited S-curve planner.
- Support non-zero start velocity and zero target velocity/acceleration initially.
- Try to meet requested time; stretch if infeasible and report `time_stretched`.
- Support replan from current planned state.

## Task 9: Mode/fault/calibration/persistence

- Implement practical mode manager and state transitions.
- Implement fault matrix and severity actions.
- Implement calibration manager state-machine skeletons.
- Implement simple flash-backed parameter-store interface with version and CRC.

## Task 10: Scheduler hooks

- Provide `MC_FastLoop_20kHz`, `MC_MotionLoop_1kHz`, and `MC_SlowLoop_10_100Hz` hooks.
- Use explicit double-buffer or copy/swap between timing domains.
- Do not block in fast/medium loops.
