# ADR-044: Persistent OD blob was truncating — enlarge to fit the full PERSIST set

- **Status:** Accepted
- **Date:** 2026-06-26
- **Relates to:** ADR-023 (persist OD gains/config), ADR-010 (persistence), ADR-040/042/043 (the entries that overflowed)

## Context

The flash payload (`MC_Params_t`, ADR-023) = the calibration subset + a serialized blob of **every
PERSIST OD entry**, gathered by `MC_Od_GatherPersistent` as `{index, sub, len, value}` records (8 B
per F32) into a fixed `MC_PARAMS_OD_BLOB_MAX` buffer. The buffer was **256 B**, and the gather
**silently `break`s when it fills** (`mc_od.c`), dropping the highest-index entries.

The motor's PERSIST set had grown to **318 B (41 entries)**, so the last 8 motor-owned entries were
never saved:

- `0x2600:6/7` — soft position limits
- `0x2700:3/4` — electrical-align current / hold
- `0x6081-5` — CiA-402 profile (velocity/accel/decel/quick-stop)

Symptom: write a value, **Save to flash**, power-cycle → the value is gone (lower-index entries
persist fine). Adding `0x2300:8` (the ADR-042 accel-ramp jerk) this session was the tipping point that
pushed `0x2600:6/7` past 256. The failure was **silent** — no error, no flag, which is what made it
hard to find. (Ruled out a mirror-trap: nothing writes `g_od.pos_limit_*` except the OD-write path and
the `LoadDefaults` seed.)

## Decision

- **`MC_PARAMS_OD_BLOB_MAX` 256 → 448 B** — fits the 318 B set with ~130 B headroom, while keeping
  `sizeof(MC_Params_t)` (~476 B) within `MC_PARAM_STORE_MAX_PAYLOAD` (512) so none of the store's
  buffers grow.
- **`MC_PARAM_STORE_VERSION` 2 → 3** — the payload layout changed, so existing flash records are
  cleanly rejected on the next boot (→ defaults → one re-save) rather than misread.
- Record the silent-truncation hazard on the buffer comment: the cap MUST stay ≥ the total PERSIST
  size; re-check it (and `sizeof(MC_Params_t) ≤ MC_PARAM_STORE_MAX_PAYLOAD`) and bump the store
  version when adding PERSIST entries.

## Consequences

- **Motor-internal** (`include/mc_calib_data.h`, `include/mc_persistent_store.h`) — NOT the shared
  Interface contract, so no `MC_IF_PROTOCOL_VERSION` bump and no CMC impact.
- **One-time:** the first boot of the new build wipes saved config (v2→v3 rejected). Re-enter +
  re-save gains / limits / cal-align once.
- The full PERSIST set now round-trips, including the soft position limits (`0x2600:6/7`) and the
  cal-align params (`0x2700:3/4`).
- `docs/spec/14_persistence.md` corrected: its "calibration-only" first-implementation note was stale
  — the OD blob has gathered config since ADR-023.

## Follow-up (not done here)

- The gather's truncation is **still silent** (it just stops at the cap). A runtime flag — set on the
  `break`, mirrored to `g_mc_debug.persist_truncated` — would make a future overflow visible in the
  watch window. Recommended the next time the persistence code is touched.

## Rejected

- **Shrink the record encoding** (drop the per-entry index/sub header) to halve overhead. Smaller but
  fragile — re-ordering/removing an entry silently corrupts the positional mapping; the keyed records
  are robust and flash has room, so just size the buffer.
- **Bump `MC_PARAM_STORE_MAX_PAYLOAD` (512) + `od_blob` to 512.** Grows three static/stack buffers in
  the store for headroom not yet needed; 448 within the existing 512-byte payload is sufficient.
