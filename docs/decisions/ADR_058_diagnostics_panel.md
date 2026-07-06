# ADR-058: Diagnostics panel — faults, fault history, operating state

- Status: Accepted
- Date: 2026-07-01
- Related: ADR-051 (NO_CONFIG), ADR-057 (NOT_HOMED), the CMC axis_manager

## Context

Fault and state info was scattered: the motor OC trip lived only in the statusword, `fault_flags` held
only NO_CONFIG/NOT_HOMED, and **nothing tracked fault history** (a fault that fired and was then cleared
left no trace). The operator had no single place to see motor + CMC health.

## Decision

- **Motor:** the OC trip becomes a `fault_flags` bit (`MC_IF_FAULT_OVERCURRENT`), so `fault_flags` is
  now the complete motor fault register. Add **`fault_flags_latched`** (`0x2600:10`, U32 RO) = sticky OR
  of `fault_flags` since boot — the fault history (which faults occurred and were cleared). Since-boot
  (not persisted). **Per-fault trigger counts (build 87):** `fault_count_no_config` / `_not_homed` /
  `_overcurrent` (`0x2600:11/12/13`, U16 RO) — how many times each `fault_flags` bit has *risen* since
  boot (saturating), RAM only.
- **PC tool:** a **"Diagnostics (faults & state)"** group on the Motor Command tab, polled at 1 Hz:
  Motor { state (statusword), active faults (`fault_flags`), fault history (`fault_flags_latched`),
  **fault counts** (`0x2600:11/12/13`) } and CMC { state (`axis_state`), error
  (`axis_error_code`/`register`), auto-cleared count (`axis_auto_fault_clears`) }, with **Refresh**
  (manual read) and **Clear-faults** buttons.

## Consequences

- Additive OD (`fault_flags_latched`, non-PDO) + a new fault bit → no `MC_IF_PROTOCOL_VERSION` bump; the
  CMC forwards like any motor entry. Persist unchanged (not persisted). Announced in CHANGELOG.
- **GUI routing fix found along the way:** read results only reach `_cmd_on_state_read` for keys listed
  in the routing sets; `home_status` (`0x2700:9`) was missing, so the homing readout was stuck at
  "RUNNING" — added it (the homing status now updates to DONE/FAILED).
- Fault history is since-boot; a lifetime (persisted) total was declined (flash wear).
