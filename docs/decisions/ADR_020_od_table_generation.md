# ADR-020: Generate the motor OD table from the shared X-macro (owner-filtered)

## Status

Accepted

## Date

2026-06-22

## Context

ADR-019 adopted Interface v2 but left the motor OD table **hand-maintained** (a static array in
`mc_od.c`), and its open question flagged the consequence: the hand table can drift silently from
the canonical map `MC_IF_OD_OBJECTS(X)`, since nothing links them at compile time. That drift was
already real — the hand table's `OD_U8`/`OD_F32` row macros hard-coded `persistent = true`, so all
four `0x2900` test-injection entries (`inject_enable/target/step_amplitude/step_trigger`) were
marked persistent, while the contract tags them `MC_IF_F_NONE` (not persistent).

## Decision

Generate `s_od_table[]` by expanding `MC_IF_OD_OBJECTS(X)`, filtered to `MC_IF_OWNER_MOTOR`
entries and bound to `g_od` by field name. The contract becomes the single source of truth:

- **Type / access** cast directly from `MC_IF_T_*` / `MC_IF_A_*` to the engine's `MC_OdType_t` /
  `MC_OdAccess_t` — the enums are value-identical (mc_if_od.h says so; two `_Static_assert`s guard
  against future drift).
- **PDO / PERSIST** derive from the contract flags (`MC_IF_F_PDO` / `MC_IF_F_PERSIST`).
- **CMC-owned `0x3xxx`** entries expand to nothing (owner dispatch) — absent from the table → the
  existing `NO_OBJECT` path.
- **`0x2A00:0` (telemetry-map count)** is the one owner=MOTOR entry handled elsewhere (mc_comms,
  REQ-0004 still deferred); it is skipped via a small name-keyed PROBE so it does not require a
  `g_od` binding.
- **`g_od.modes_display` → `g_od.modes_of_operation_display`** so the name matches the contract and
  the `&g_od.<name>` binding resolves.
- **Write-range windows** (5 entries: `motor_pole_pairs`, `est_use_observer`, the three ranged
  `inject_*`) are motor policy — not in the contract — so they are applied in `MC_Od_Init`; the
  table is therefore non-`const` (RAM-resident).

**Drift is now a compile error**: a motor-owned contract entry whose `name` has no matching `g_od`
field fails to compile (`&g_od.<name>`); type/access/PDO/PERSIST follow the contract automatically.

## Verification

Host build of the **original** (git HEAD) and **generated** tables, each dumped (index, sub, type,
access, pdo, persist, min, max, data!=NULL) and diffed:

- Both **62 entries**; every field byte-identical **except** the four `0x2900` `persistent` flags
  flipping `1 → 0` — i.e. the generated table corrects the latent drift to match the contract.
- The **used** flag `pdo_mappable` (consumed by `mc_comms` telemetry-map validation) is identical
  everywhere; `persistent` is **currently unused** (no code reads it — persistence is calib-only,
  ADR-010), so the correction is behaviourally inert today.
- `gcc -fsyntax-only` of `mc_comms.c` + `mc_od.c` against the v2 headers is clean.

## Consequences

- **Latent bug fixed:** `inject_*` (0x2900) are no longer marked persistent — correct (a test
  injection must not survive a power cycle) and contract-aligned. Inert until OD-driven persistence
  is wired, but now correct by construction.
- **Table moved flash → RAM** (~2.7 KB) to allow the init-time range patch. Trivial on the G474.
- **Single remaining hand-bound special:** `0x2A00` (REQ-0004). If/when it moves into the OD, the
  skip marker is removed and the generation covers it too.
- **Resolves ADR-019's open question.** Future contract changes to motor-owned entries now surface
  at compile time instead of silently diverging.

## Files affected

- src/mc_od.c (table now generated; range patch in MC_Od_Init)
- include/mc_od_store.h (`modes_display` → `modes_of_operation_display`)
- src/mc_scheduler.c (rename references)
- docs/decisions/ADR_020_od_table_generation.md, docs/decisions/ADR_000_decision_log.md, ADR_019 (open question resolved)

## Open questions

- `0x2A00` telemetry map into the OD (REQ-0004) remains the one deferred special-case binding.
