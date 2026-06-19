# Architecture Decision Log

This file is the index of all major architecture and implementation decisions.

Update this file whenever a design decision is made, changed, clarified, or discovered
during implementation. For larger decisions, also create a separate `ADR_NNN_*.md` file.

| ADR | Date | Status | Title | Summary | Files affected |
|---:|---|---|---|---|---|
| 000 | 2026-06-19 | Accepted | Decision log process | The coding agent records architectural decisions in docs/decisions before or alongside code changes. | docs/decisions, requirements.yaml, docs/spec, include |
| 001 | 2026-06-19 | Accepted | Scope: motor-control MCU only; freeze SPI seam | Build only the motor-control MCU; fully specify the inter-MCU SPI link as a frozen boundary contract; network MCU + external protocol are a separate project. | ADR_001, docs/spec/06_spi_protocol.md, include/mc_spi_protocol.h, requirements.yaml |
| 002 | 2026-06-19 | Accepted | Clean-room reimplementation with golden-reference validation | Reimplement modules from the specs; validate each against bldc_axis_controller as a golden reference; verified on-target via SWD live watch (ADR-005). | ADR_002, docs/spec/15_basic_tests.md, tests/, requirements.yaml |
| 003 | 2026-06-19 | Accepted | Velocity feedback: position-tracking observer default | State-estimator default velocity = encoder position-tracking observer (Ellis Fig 18-11); finite-difference+LPF as runtime-selectable fallback. | ADR_003, docs/spec/11_feedback_and_encoders.md, include/mc_state_estimator.h, requirements.yaml |
| 004 | 2026-06-19 | Accepted | Per-board hardware config + generic motor model | Introduce MC_BoardConfig_t and MC_MotorModel_t; hardware values become per-board config; ship board profile 0 from the proven reference board. | ADR_004, docs/spec/17_board_and_motor_config.md, include/mc_board_config.h, include/mc_motor_model.h, requirements.yaml |
| 005 | 2026-06-19 | Accepted | Bring-up & verification via on-target SWD live watch | Primary verification is incremental on-target bring-up over SWD using the debugger live watch window + volatile debug-mirror/inject structs; host Unity tests optional. | ADR_005, docs/spec/15_basic_tests.md, CLAUDE.md, requirements.yaml |
| 006 | 2026-06-19 | Accepted | Real-time scheduling & ISR dispatch | Fast 20 kHz from ADC end-of-conversion (ADC triggered by TIM1 TRGO=OC4REF at the PWM peak; sample-synchronised), medium 1 kHz from TIM7, slow 100 Hz decimated/serviced in main loop; HAL-free mc_scheduler + main.c dispatch. | ADR_006, include/mc_scheduler.h, src/mc_scheduler.c, src/mc_debug.*, Core/Src/main.c, docs/spec/04_realtime_scheduling.md |
