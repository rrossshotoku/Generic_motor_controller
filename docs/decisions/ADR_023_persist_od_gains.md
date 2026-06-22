# ADR-023: Persist OD gains — wire the `persistent` flag into the flash store

## Status

Accepted

## Date

2026-06-22

## Context

OD entries flagged `MC_IF_F_PERSIST` (the tunable gains/config: velocity/position/FOC/observer
gains, motor model, current trip, profile params) advertised persistence, but the flag was
**descriptive only — nothing consumed it** (`grep` for `->persistent` = 0 hits). The flash store
saved just `MC_CalibData_t` (the calibration subset), so on boot `g_od` was seeded from hard-coded
defaults (`MC_OdStore_LoadDefaults`) and any runtime gain change was lost at power-off. This is the
gap ADR-010 anticipated ("the calibration subset of the eventual full `MC_Params_t`"); the user hit
it directly — `vel_kp` didn't survive a power cycle.

User decision: **explicit SAVE button** (not auto-save), to spare flash endurance.

## Decision

Make the `persistent` flag real:

- **OD serialization** (`mc_od.c`): `MC_Od_GatherPersistent` walks the table and serializes each
  persistent entry as `{index(LE16), subindex, len, value}`; `MC_Od_RestorePersistent` parses that
  and writes each back via `MC_Od_Write`. Records whose index/sub is unknown or whose size no longer
  matches are skipped — so it degrades gracefully if the persistent set changes between builds.
- **Store payload** grows from bare `MC_CalibData_t` to `MC_Params_t { MC_CalibData_t calib;
  uint16_t od_blob_len; uint8_t od_blob[256]; }`. `MC_PARAM_STORE_VERSION` 1 → 2;
  `MC_PARAM_STORE_MAX_PAYLOAD` 256 → 512 (fits the 2 KB A/B slot).
- **`params_save()`** (was `calib_save`) gathers calib + the persistent OD entries into `MC_Params_t`
  and latches a save. Triggered by the existing **SAVE command** (`0x2800:1 = MC_IF_SAVE_MAGIC`) and
  also by the align / mech-zero captures (they persist the full set).
- **Boot**: load `MC_Params_t` → apply calibration (as before) + `MC_Od_RestorePersistent` into
  `g_od`; the slow loop's `od_apply_gains` then pushes the restored gains to the live controllers.

## Trigger & timing

- **Explicit save.** Editing a gain via OD write takes effect *live* immediately (`od_apply_gains`);
  it persists only when you issue SAVE. No auto-save-on-write (that would thrash a ~10k-cycle flash;
  a streaming GUI slider could exhaust a page).
- **Idle-gated flash write** (unchanged, ADR-010): the erase/program runs only in the slow loop with
  the power stage **off**. A SAVE issued while driving is latched and commits when the drive goes
  idle. Flash program stalls the flash interface and the 20 kHz/1 kHz loops run from flash, so this
  gating prevents any mid-drive timing disruption; at idle, at worst the background sensor loops
  pause for the ~tens-of-ms write and resume (nothing is being controlled, so it's harmless).

## Consequences

- Gains now survive a power cycle once SAVE is issued — closes the `vel_kp`-doesn't-persist gap.
- **Store-format change → version bump.** The old v1 record (bare calibration) is invalidated on
  first boot of v2, so after flashing you re-run the current-offset + electrical-align captures and
  **re-set the mech zero** once. (A migration that preserves v1 calibration was considered and
  declined — not worth it mid-bench-bringup.)
- The `od_blob` is sized (256 B) for the current persistent set (~219 B gathered) with headroom; if
  the persistent set grows past ~256 B the gather truncates the tail — raise `MC_PARAMS_OD_BLOB_MAX`
  (and `MC_PARAM_STORE_MAX_PAYLOAD`) if so.

## Verification

- Host round-trip (`gcc`): set `vel_kp/foc_id_kp` (F32), `motor_pole_pairs` (U16),
  `est_use_observer` (U8) → gather → `MC_Od_Init` (reset to defaults) → restore → **all four recover
  exactly** (219 bytes gathered). `gcc -fsyntax-only` of `mc_scheduler.c` + `mc_od.c` clean.
- **On-target (pending)**: change `vel_kp` via the PC tool, write `0x2800:1 = 0x7376`, stop the
  drive (so the write commits), power-cycle → `vel_kp` retains its value.

## Files affected

- src/mc_od.c (+ include/mc_od.h): `MC_Od_GatherPersistent` / `MC_Od_RestorePersistent`
- include/mc_calib_data.h: `MC_Params_t`
- include/mc_persistent_store.h: store version 2, max payload 512
- src/mc_scheduler.c: `calib_save` → `params_save` (gather gains), boot restore, `<string.h>`
- docs/decisions/ADR_023_persist_od_gains.md, docs/decisions/ADR_000_decision_log.md, docs/spec/05_object_dictionary.md

## Open questions

- Optional future: debounced **auto-save** on persistent-entry write (mark-dirty + flush when idle),
  if the explicit-save workflow proves tedious.
- A v1→v2 migration if preserving existing calibration across the bump ever matters.
