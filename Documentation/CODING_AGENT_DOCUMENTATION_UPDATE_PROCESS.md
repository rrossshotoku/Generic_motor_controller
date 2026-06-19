# Coding Agent Documentation Update Process

## Purpose

This document tells the coding agent how to record architectural decisions, implementation changes, and documentation updates while developing the motor-control framework.

The goal is to keep the project easy to regenerate into full documentation later, including:

- Word architecture documents
- Markdown specifications
- Doxygen pages
- C header documentation
- Graphviz diagrams
- Requirements summaries
- Implementation notes

The coding agent must not rely on Word or PDF documents as the source of truth. Plain-text project files are the source of truth.

---

## Core Rule

Whenever a design decision is made, changed, clarified, or discovered during implementation, the coding agent must update the decision records before or alongside the code change.

Do not only implement the decision in C code.

Do not leave architectural decisions hidden in chat history.

Do not rely on comments inside source files as the only record of the decision.

---

## Source-of-Truth Order

Treat the project files in this priority order:

1. `docs/decisions/*.md`  
   Records why architectural decisions were made and what changed.

2. `requirements.yaml`  
   Machine-readable summary of the current agreed requirements.

3. `docs/spec/*.md`  
   Human-readable implementation specifications for each subsystem.

4. `include/*.h`  
   C interface contracts and Doxygen comments.

5. `src/*.c`  
   Implementation files.

If these disagree, stop and flag the inconsistency before making further changes.

---

## Required Decision Log Files

The repository should contain:

```text
docs/
    decisions/
        ADR_000_decision_log.md
        ADR_001_example_decision.md

    spec/
        architecture.md
        object_dictionary.md
        spi_protocol.md
        control_loops.md
        trajectory_planner.md
        foc_backend.md
        faults.md
        calibration.md
```

### `ADR_000_decision_log.md`

This is the master index of all important decisions.

Update this file whenever a decision is made or changed.

Each entry should be short, but must include:

- ADR number or decision ID
- Date
- Status
- Short title
- One-sentence summary
- Files affected

### Separate ADR files

For small decisions, updating `ADR_000_decision_log.md` is enough.

For larger decisions, create a separate ADR file:

```text
docs/decisions/ADR_004_change_trajectory_planner_policy.md
```

Create a separate ADR when the decision affects one or more of:

- Architecture diagrams
- Public C interfaces
- Object dictionary layout
- SPI protocol
- Timing-domain behaviour
- Control-loop structure
- Fault handling
- Calibration procedure
- Persistent storage format
- Motor backend abstraction
- Encoder abstraction

---

## What Counts as an Architectural Decision?

Update the decision log for changes such as:

- Changing command paths, such as joystick bypassing the position loop
- Changing loop rates or scheduling ownership
- Adding, removing, or renaming a module
- Changing a C public interface in `include/*.h`
- Changing object dictionary indexes, types, scaling, or access rights
- Changing SPI packet format or message semantics
- Changing the trajectory planner algorithm or replan policy
- Changing PID/PI controller behaviour, anti-windup, or saturation policy
- Changing FOC conventions, angle sign, current sign, transform convention, or PWM convention
- Changing calibration sequence or stored calibration data
- Changing fault severity, fault action, or reset behaviour
- Changing persistent storage layout, versioning, or CRC behaviour
- Adding support for a new motor backend or encoder backend

Do not update the decision log for trivial local implementation details that do not affect interfaces or architecture, such as renaming a local variable or fixing a typo.

---

## Required Update Sequence

When a decision is made, the coding agent should follow this sequence:

1. Update or create the relevant ADR file in `docs/decisions/`.
2. Update `docs/decisions/ADR_000_decision_log.md`.
3. Update `requirements.yaml` if the decision changes machine-readable requirements.
4. Update the relevant `docs/spec/*.md` subsystem document.
5. Update the relevant `include/*.h` public interface if required.
6. Update the relevant `src/*.c` implementation.
7. Update diagrams in `diagrams/*.dot` if the architecture/data flow changes.
8. Run or update basic module tests where available.
9. Summarise the change in the coding-agent response.

---

## ADR Template

Use this template for every separate ADR file.

```md
# ADR-XXX: Decision title

## Status

Proposed / Accepted / Superseded

## Date

YYYY-MM-DD

## Context

Describe the problem, uncertainty, limitation, or implementation finding that led to this decision.

## Decision

State the decision clearly and directly.

## Reasoning

Explain why this option was chosen.

Mention alternatives that were rejected if relevant.

## Consequences

Describe what this affects:

- Architecture
- C interfaces
- Object dictionary
- SPI protocol
- Timing domains
- Control loops
- Fault behaviour
- Calibration
- Persistent storage
- Tests
- Documentation

## Files affected

- docs/spec/example.md
- include/example.h
- src/example.c
- requirements.yaml
- diagrams/example.dot

## Open questions

List unresolved points, if any.
```

---

## Example ADR

```md
# ADR-006: Joystick mode bypasses trajectory and position control

## Status

Accepted

## Date

2026-06-19

## Context

Joystick operation should feel direct and should not be treated as a profile-position move.

## Decision

Joystick mode shall bypass the trajectory planner and position controller. Joystick input shall pass through deadband, scaling, acceleration limiting, and soft-limit slowdown, then command the velocity controller directly.

## Reasoning

This preserves the nested servo structure while giving joystick control a direct velocity-command path. It also avoids forcing operator velocity commands through the profile-position trajectory planner.

## Consequences

- The mode manager must select a separate joystick velocity command path.
- The trajectory planner is inactive in joystick mode.
- The position controller is bypassed in joystick mode.
- Soft limits still apply through the joystick conditioning block.
- Joystick release shall use controlled deceleration and then enter position hold.
- The architecture diagram must show the bypass path.

## Files affected

- docs/spec/architecture.md
- docs/spec/control_loops.md
- include/mc_mode_manager.h
- include/mc_joystick.h
- requirements.yaml
- diagrams/command_paths.dot

## Open questions

None.
```

---

## `ADR_000_decision_log.md` Format

Use this format for the master decision log:

```md
# Architecture Decision Log

This file is the index of all major architecture and implementation decisions.

| ADR | Date | Status | Title | Summary | Files affected |
|---:|---|---|---|---|---|
| 006 | 2026-06-19 | Accepted | Joystick mode bypasses trajectory and position control | Joystick is conditioned then sent directly to the velocity controller. | architecture.md, control_loops.md, mc_mode_manager.h, mc_joystick.h |
```

---

## Updating `requirements.yaml`

Update `requirements.yaml` when a decision changes a durable requirement.

Examples:

```yaml
control_paths:
  joystick_velocity:
    bypasses_trajectory_planner: true
    bypasses_position_controller: true
    conditioning:
      deadband: true
      scaling: true
      acceleration_limit: true
      soft_limit_slowdown: true
```

```yaml
trajectory_planner:
  type: jerk_limited_s_curve
  target_time_required: true
  allow_time_stretch: true
  report_time_stretch: true
```

```yaml
motor_backends:
  initial: bldc_pmsm_foc_3shunt
  future_supported:
    - brushed_dc_hbridge
```

---

## Updating Markdown Specs

When a decision affects a subsystem, update the matching file under `docs/spec/`.

Examples:

- `docs/spec/architecture.md` for block diagrams and high-level paths
- `docs/spec/control_loops.md` for position/velocity/current loops
- `docs/spec/object_dictionary.md` for OD changes
- `docs/spec/spi_protocol.md` for inter-MCU protocol changes
- `docs/spec/trajectory_planner.md` for trajectory/replan changes
- `docs/spec/foc_backend.md` for FOC implementation changes
- `docs/spec/faults.md` for fault actions and severity changes
- `docs/spec/calibration.md` for calibration state-machine changes

The spec should describe the current accepted design, not every rejected alternative. Rejected alternatives belong in the ADR.

---

## Updating C Headers

When a decision changes a public API, update the corresponding header in `include/`.

Public header comments must remain Doxygen-compatible.

Every public type, enum, struct, and function should have a clear comment.

Example:

```c
/**
 * @brief Selects the active command path for the motor axis.
 *
 * The mode manager is responsible for routing profile-position commands through
 * the trajectory and position loops, while joystick velocity commands bypass the
 * trajectory and position controllers and enter the velocity loop after joystick
 * conditioning.
 */
void MC_ModeManager_Update(MC_ModeManager_t *manager,
                           const MC_ModeInputs_t *inputs,
                           MC_ModeOutputs_t *outputs);
```

---

## Updating Diagrams

If the data flow, block structure, timing domains, or command paths change, update the relevant Graphviz files in `diagrams/`.

Examples:

```text
diagrams/architecture.dot
diagrams/command_paths.dot
diagrams/timing_domains.dot
diagrams/spi_protocol.dot
```

Do not leave diagrams stale after changing architecture.

---

## Coding Agent Behaviour Rules

The coding agent shall:

1. Keep all motor-control framework code in `mc_*` modules.
2. Avoid modifying CubeMX-generated code except inside `USER CODE` sections.
3. Keep internal units in SI units unless the relevant spec says otherwise.
4. Keep object dictionary scaling/conversion at the OD boundary.
5. Keep BLDC/FOC-specific concepts inside the BLDC/FOC backend.
6. Keep future brushed-DC support possible by preserving generic torque/current request interfaces.
7. Keep SSI and quadrature encoder hardware behind the common position-feedback interface.
8. Compile after each meaningful implementation stage where possible.
9. Update documentation before or alongside architecture-affecting code changes.
10. Stop and report if the implementation requires a hardware-specific decision not present in the specification.

---

## Change Summary Required After Each Coding Session

At the end of each coding session, the coding agent should produce a short summary:

```md
## Coding Session Summary

### Implemented

- ...

### Decisions made

- ADR-XXX: ...

### Files changed

- ...

### Tests run

- ...

### Open questions / blocked items

- ...
```

This summary helps regenerate the full documentation later.

---

## Regenerating Full Documentation Later

When the updated repo is given back for documentation generation, the documentation generator should read, in order:

1. `docs/decisions/*.md`
2. `requirements.yaml`
3. `docs/spec/*.md`
4. `include/*.h`
5. `diagrams/*.dot`
6. Important implementation notes from `src/*.c`

The generated documentation should reflect the accepted ADRs and current C interfaces, not the original Word document if the project has moved on.
