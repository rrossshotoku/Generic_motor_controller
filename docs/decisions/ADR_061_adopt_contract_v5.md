# ADR-061: Adopt shared contract v5 (dual-bootloader Phase 1) on the motor

- Status: Accepted
- Date: 2026-07-03
- Related: ADR-019 (OD generation), REQ-0015, contract CHANGELOG [5.0.0]

## Context

The shared Interface advanced to **v5** (dual-bootloader Phase 1, wire-breaking): `MC_IF_PROTOCOL_VERSION`
4 → 5, four bootloader-owned OD entries (`0x1F5x`), three segmented-download message types
(`0x14/0x15/0x16`), a new owner `MC_IF_OWNER_BOOTLOADER`, node state `MC_IF_NODE_BOOTLOADER` (`0x07`),
and four result codes (`0x09`–`0x0C`). A v5 CMC `BAD_VERSION`s a v4 motor, so the SPI OD link is dead
until the motor rebuilds against v5. The Interface also moved to its new canonical home under
`Generic_axis_controller` (see [[interface-gui-location]]).

## Decision

Adopt v5 on the motor **app** (REQ-0015 items 1–3; the bootloader binary is Phase 2, deferred):
- The OD generator (`mc_od.c`) dispatches per owner via `OD_ROW_MC_IF_OWNER_<owner>`. Added an
  **`OD_ROW_MC_IF_OWNER_BOOTLOADER` skip** (like CMC) so the app compiles against v5 and does **not**
  serve the `0x1F5x` entries — they belong to the separate bootloader binary. Without it the X-macro
  build fails (`'program_data' undeclared`).
- Repointed the build (`.cproject`) + source-comment include path from `../Lightweight_CMC/Interface`
  to `../Generic_axis_controller/Generic_axis_controller/Interface` (the moved canonical v5 location).
- The app's default message dispatch already returns `MC_IF_ERR_UNKNOWN_MSG` for unknown types, which
  covers the three new download messages; the new node-state + result-code enums are additive (the app
  doesn't implement them). Host-compiles clean against v5. `fw_build 83`.

## Consequences

- No motor logic change beyond the skip macro; a v5 CMC frame now version-matches. Bootloader-owned
  entries are absent from the app OD (→ `NO_OBJECT`), as intended.
- **Phase 2 remains open** (REQ-0015): the motor bootloader binary — `0x1F5x` dispatch, segmented-SDO
  receiver, flash programming, reset-marker trigger, shared `motor_spi`/`flash` BSP.
- The old `../Lightweight_CMC/Interface` path is legacy; the motor now builds against the moved contract.
