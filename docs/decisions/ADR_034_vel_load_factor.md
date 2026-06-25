# ADR-034: vel_load_factor — operator load multiplier on velocity-loop kp/ki (REQ-0014)

- **Status:** Accepted
- **Date:** 2026-06-25
- **Related:** REQ-0014 (CMC→motor), ADR-012 (velocity loop / torque model), ADR-031 (velocity_ff_gain — same "live multiplier" pattern)

## Context

REQ-0014 (raised by the CMC): the CMC web config page has a **load-factor slider** (0.3 "light load" →
2.0 "heavy load") so the operator scales the velocity-loop response to the current payload — heavier camera
body + lens combos need higher loop gain to track without lag; lighter setups need less to avoid
noise/oscillation. It was previously a **CMC-only placeholder** (`0x3040 axis_payload_weight_kg`, CHANGELOG
[3.6.0]) that never reached the loop math. It now becomes a **motor-owned dimensionless multiplier** the
velocity loop consumes directly. The CMC removes `0x3040` in the same migration.

## Decision

Add motor-owned **`vel_load_factor` (0x2300:5, F32, RW, PERSIST, default 1.0)**. The velocity loop applies
it at runtime, in `od_apply_gains` where the live gains already flow to the PID config:

```c
s_vel_cfg.pid.kp = g_od.vel_kp * factor;   /* factor = clamp(vel_load_factor, 0.3, 2.0) */
s_vel_cfg.pid.ki = g_od.vel_ki * factor;   /* kd unchanged */
```

The base `vel_kp` / `vel_ki` (`0x2300:1/2`) stay the **operator-tuned baseline**; `vel_load_factor` is the
live load adjustment on top. **1.0 = current behaviour.**

- **Clamped, not rejected.** The factor is clamped to **[0.3, 2.0]** at apply time (matching the CMC
  slider). Clamp-not-reject is consistent with the `velocity_ff_gain` (≥0) and `current_trip_a` (floor)
  patterns — a stray or out-of-range write can't zero or blow up the loop gain, and the OD write path
  stays a generic typed write (no per-entry range metadata needed).
- **Default 1.0** seeded in `LoadDefaults` so a fresh contract update / old flash is behaviour-neutral.
- **PERSIST** — saved alongside the other `0x2300` gains; survives reboot.

Additive `0x2300` gain entry, non-PDO → **`MC_IF_PROTOCOL_VERSION` stays 4**. CHANGELOG **[4.1.0]**, which
also records the CMC-side `0x3040` removal — it is one migration.

## Consequences

- The CMC's existing slider + `axis_manager` get/set start working (writes to `0x2300:5` no longer
  `NO_OBJECT`).
- The `vel_kp` / `vel_ki` tuning workflow (PC tool on `0x2300:1/2`) is unaffected — those remain the
  baseline; the load factor multiplies them.
- `0x3040 axis_payload_weight_kg` (CMC-owned placeholder) is removed — logged in [4.1.0]; removing an
  acyclic OD object changes no fixed wire layout, so no version bump (INTERFACE_SPEC §6).

## Files

`../Lightweight_CMC/Interface/mc_if_od.h` + `CHANGELOG.md` + `REQUESTS.md` (REQ-0014); `include/mc_od_store.h`,
`src/mc_od.c`, `src/mc_scheduler.c` (`od_apply_gains`); `docs/spec/09_control_loops.md`,
`ADR_000_decision_log.md`, `requirements.yaml`.
