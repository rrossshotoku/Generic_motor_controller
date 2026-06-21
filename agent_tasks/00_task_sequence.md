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

## Task 11: Bring OD/SPI in line with the Interface contract

Tracking: `../Lightweight_CMC/Interface/REQUESTS.md` REQ-0001 .. REQ-0006.
Triggered when the network MCU's OD-over-UDP bridge (Phase 5 of `Lightweight_CMC`) shipped
and the audit identified the gap. ADR-015 already anticipated this work as the deferred
part of its scope.

- REQ-0001 (blocking) — add the 18 CiA-402 standard OD entries (0x1000, 0x1001, 0x603F,
  0x6040, 0x6041, 0x6060, 0x6061, 0x607A, 0x6064, 0x6081, 0x6083, 0x6084, 0x6085, 0x60FF,
  0x606C, 0x6071, 0x6077). Read/write callbacks route to the owning module (mode manager
  for controlword/statusword/mode; position/velocity controllers for targets/actuals;
  faults for error_register/error_code). Scale via `MC_IF_POS_SCALE` / `MC_IF_VEL_SCALE`
  / `MC_IF_CUR_SCALE` at the OD boundary.
- REQ-0002 (blocking) — extend `MC_OdStatus_t` with `MC_OD_ERR_NO_SUB` and
  `MC_OD_ERR_NOT_READY`; update the wire-mapping table in `src/mc_comms.c`.
- REQ-0003 — add 6 missing manufacturer entries (`0x2000:3,4`, `0x2600:1`, `0x2700:2`,
  `0x2800:2,3`).
- REQ-0004 — move `0x2A00` telemetry-map into the OD table with callbacks (currently a
  special case in `mc_comms.c`).
- REQ-0005 — stage `ERROR` messages on frame-validation failure instead of silently
  returning telemetry.
- REQ-0006 — remove dead `include/mc_spi_protocol.h` types, `MC_SPI_PROTOCOL_VERSION`,
  `MC_SPI_MAX_PAYLOAD` from `include/mc_config.h`.

Close each REQ in `REQUESTS.md` as you complete it. The work updates ADR-015 (already has
a *Cross-project status update* section) and `docs/spec/05_object_dictionary.md` (already
has a *Status* banner pointing here).
