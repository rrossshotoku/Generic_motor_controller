# ADR-051: Fail-safe when the persistent config doesn't load

- Status: Accepted
- Date: 2026-06-30
- Related: ADR-010 (A/B record store), ADR-026 (calibration completeness), ADR-038 (cold-boot anchor)

## Context

At boot, `MC_Framework_Init` reads the whole persistent blob once (calibration + all PERSIST OD
entries) through the A/B, CRC32-validated store and applies it. If the read returned not-OK (no
valid record — a never-configured board, both slots corrupt, or a version-bumped struct), the
original code fell through **silently** to compile-time defaults — and the operational drive could
still be enabled with a **default electrical offset (0)**, which mis-commutates (the wrong-direction
/ runaway seen at cold boot).

The store read itself is already robust: two slots, CRC32, version-gated, and the internal flash is
ready by the time `Init` runs (latency is set in `HAL_Init`, well before). So the real gap is
**fail-safe behaviour**, not the read mechanism.

## Decision

1. **Retry the load** (`MC_STORE_LOAD_ATTEMPTS = 3`) so an unforeseen transient can't latch
   defaults — belt-and-suspenders over the already-redundant store.
2. **Inhibit the operational drive when there's no valid config.** The medium loop sets
   `fs.severe_active |= !MC_PersistentStore_HasValid()`, so the mode manager won't enable the drive.
   Commissioning paths (watch-inject, dq-test/align) bypass the mode manager and stay usable — so a
   fresh board can still be aligned, saved, then driven.
3. **Raise an OD bit.** `od_apply_gains` sets `0x2600:1 fault_flags` bit `MC_IF_FAULT_NO_CONFIG`
   (`0x1`) whenever no valid config is held; it self-clears once a valid record loads or is saved.

`HasValid` is the gate signal: the store's version-gating means `HasValid == true` implies a
struct-matching, applicable record, so a "valid-but-unreadable" state doesn't occur in practice.

## Consequences

- A fresh / un-configured board boots with the drive inhibited + the fault bit set — correct (no
  valid calibration). Commission via the inject/align path, save, and the gate clears.
- Contract: a new `MC_IF_FAULT_NO_CONFIG` semantic on the existing `0x2600:1 fault_flags` (a bit
  definition, not a new entry) → no `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected. Logged in CHANGELOG.
- This catches the case where the config genuinely didn't load. If a cold-boot symptom is instead a
  **position-estimator init-order race** (config loaded fine but the angle was wrong), that's a
  separate issue — still open in the cold-boot watch note.
