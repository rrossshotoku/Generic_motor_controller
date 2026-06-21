# Object Dictionary Specification

> **Status, 2026-06-21 (ADR-017)**: `mc_od.c` now implements the manufacturer `0x2xxx` set
> **and** the 17 CiA-402 standard `0x1xxx`/`0x6xxx` objects (REQ-0001/0003 done) — RO actuals
> are mirrored from live state, scaled to wire units; RW objects are stored for the mode manager
> (E1) to consume. Result codes match the wire (REQ-0002). The only canonical entry still
> outside the OD table is the **telemetry map `0x2A00`** (REQ-0004, deferred — handled in
> `mc_comms` for now). Authoritative map: `../../../Lightweight_CMC/Interface/mc_if_od.h`
> (`MC_IF_OD_OBJECTS`). Request log: `../../../Lightweight_CMC/Interface/REQUESTS.md`.


## OD implementation type

Use a static object table with typed read/write functions.

Each object entry shall define:

- Index
- Subindex
- Name
- Data type
- Access rights
- Scaling/unit
- Default value
- Min/max
- PDO mappable flag
- Persistent flag
- Optional read callback
- Optional write callback

## OD API requirements

- Lookup by index/subindex.
- Type checking.
- Range checking.
- Access checking.
- Callback support for live values and side effects.
- No dynamic registration in first implementation.

## Initial object groups

### CiA 402-style core

| Index | Sub | Name | Type | Access | Notes |
|---:|---:|---|---|---|---|
| 0x6040 | 0 | Controlword | U16 | RW | Drive command bits |
| 0x6041 | 0 | Statusword | U16 | RO | Drive status bits |
| 0x6060 | 0 | Modes of operation | I8 | RW | Requested mode |
| 0x6061 | 0 | Modes of operation display | I8 | RO | Active mode |
| 0x603F | 0 | Error code | U16 | RO | Primary active error |
| 0x1001 | 0 | Error register | U8 | RO | Summary fault bits |

### Position/profile objects

| Index | Sub | Name | Type | Access | Internal conversion |
|---:|---:|---|---|---|---|
| 0x607A | 0 | Target position | I32 | RW | scaled -> rad |
| 0x6064 | 0 | Position actual value | I32 | RO | rad -> scaled |
| 0x6081 | 0 | Profile velocity | U32 | RW | scaled -> rad/s |
| 0x6083 | 0 | Profile acceleration | U32 | RW | scaled -> rad/s^2 |
| 0x6084 | 0 | Profile deceleration | U32 | RW | scaled -> rad/s^2 |
| 0x6085 | 0 | Quick stop deceleration | U32 | RW | scaled -> rad/s^2 |

### Velocity objects

| Index | Sub | Name | Type | Access | Internal conversion |
|---:|---:|---|---|---|---|
| 0x60FF | 0 | Target velocity | I32 | RW | scaled -> rad/s |
| 0x606C | 0 | Velocity actual value | I32 | RO | rad/s -> scaled |

### Torque/current objects

| Index | Sub | Name | Type | Access | Notes |
|---:|---:|---|---|---|---|
| 0x6071 | 0 | Target torque/current | I16/I32 | RW | scaled torque/current demand |
| 0x6077 | 0 | Torque/current actual | I16/I32 | RO | measured/estimated |

### Manufacturer-specific initial ranges

| Range | Purpose |
|---:|---|
| 0x2000-0x20FF | Axis configuration and units/scaling |
| 0x2100-0x21FF | Trajectory planner configuration/status |
| 0x2200-0x22FF | Position controller gains/status |
| 0x2300-0x23FF | Velocity controller gains/status |
| 0x2400-0x24FF | Current/FOC gains/status |
| 0x2500-0x25FF | Encoder and state estimator configuration |
| 0x2600-0x26FF | Faults, limits, derating, diagnostics |
| 0x2700-0x27FF | Calibration commands/status/results |
| 0x2800-0x28FF | Persistent store commands/status |
| 0x2900-0x29FF | Commissioning/test injection |

## Side-effect rules

- Writing controlword updates the mode-manager command latch, not the controllers directly.
- Writing gains updates inactive/shadow config and is applied at safe update points.
- Writing calibration commands starts calibration only if mode and safety preconditions allow it.
- Writing save command requests slow-loop persistent-store operation.

## Authoritative object map (ADR-013)

The concrete OD object list is the `MC_IF_OD_OBJECTS(X)` X-macro in the shared interface package
`../Lightweight_CMC/Interface/mc_if_od.h` (both MCUs generate their tables from it). CiA-402
standard objects (0x1xxx/0x6xxx) are scaled integers (factors in that header); manufacturer
objects (0x2xxx: gains, telemetry, calibration, persistence, test-injection) are **float32 SI**.
The motor MCU's `mc_od` table is generated from that list, mapping each entry to a live
variable / shadow config / callback.

### Implementation status (ADR-015)

`mc_od.c` implements the engine (typed Find/Read/Write with access/type/size/range checks +
callbacks) and a **tuning-focused subset** of the map bound to the `g_od` backing store
(`mc_od_store.h`): gains, key telemetry, limits, motor params (manufacturer objects as float32
SI). The scheduler seeds it, applies OD-written gains to the live controllers in the slow loop,
and mirrors telemetry in the medium loop. The CiA-402 standard objects (0x6xxx, with scaling)
and the 0x2A00 telemetry map are implemented alongside the SPI-slave transport and the mode
manager.
