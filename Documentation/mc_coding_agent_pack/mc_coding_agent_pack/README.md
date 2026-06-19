# Motor Controller Coding-Agent Pack

This pack is the preferred input format for an AI coding agent. It is intentionally plain text and source-oriented, rather than Word/PDF.

## Read order for coding agents

1. `AGENT_README_FIRST.md`
2. `agent_tasks/00_task_sequence.md`
3. `docs/spec/00_system_summary.md`
4. `docs/spec/01_architecture.md`
5. `docs/spec/02_interfaces_and_units.md`
6. The specific subsystem file for the module being implemented
7. Matching header in `include/`
8. Matching stub in `src/`

## Target

- Language: C
- MCU: STM32G474RET3
- Project basis: STM32CubeMX-generated project supplied by the user
- Initial motor backend: three-shunt BLDC/PMSM FOC
- Future backend: brushed DC/H-bridge, so outer motion layers must remain motor-type independent
- Internal units: SI physical units
- Documentation: Doxygen-compatible comments plus Markdown specifications

## Key rule

Do not implement everything in one pass. Implement one module at a time, compile frequently, and keep CubeMX-generated code isolated from framework code.
