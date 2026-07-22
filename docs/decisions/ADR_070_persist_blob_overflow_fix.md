# ADR-070: PERSIST blob overflow dropped position_recall_enable — grow buffer, backward-compatibly

- Status: Accepted
- Date: 2026-07-22
- Related: ADR-044 (the same bug at 256→448 B), ADR-023/010 (persistence), ADR-067 (position recall)

## Symptom

Setting `position_recall_enable` (0x2700:11) ON and saving to flash did not persist: after a reboot
it read back OFF, so the axis always came up NOT_HOMED even though homing worked within a session.

## Root cause — the ADR-044 truncation bug, resurfaced

The flash payload serialises every motor-owned `MC_IF_F_PERSIST` OD entry into a fixed
`od_blob[MC_PARAMS_OD_BLOB_MAX]` (448 B) as `{index, sub, len, value}` TLV records.
`MC_Od_GatherPersistent` iterates the OD table in order and **silently `break`s when the buffer is
full**, dropping the highest-index entries. Since ADR-044 raised the cap to 448 B, the motor PERSIST
set grew with thermal (×4), dither (×4), position-recall, and current-demand-limit entries to
**473 B — 25 B over**. The first casualties past 448 B are `0x2700:11 position_recall_enable` (ends
at 452 B) and the three `0x2930` notch entries. So the enable flag was never written; on boot it
restored to its default (0 = off) → recall disabled → NOT_HOMED. Classic silent overflow: every
individual save "succeeded".

## Decision

Grow the buffer, **backward-compatibly**, so no configuration is lost — critically the electrical
alignment offset, which cannot currently be regenerated (rotor loaded).

- `MC_PARAMS_OD_BLOB_MAX` **448 → 640 B** (~167 B / ~20 F32 headroom over the current 473 B).
- `MC_PARAM_STORE_MAX_PAYLOAD` **512 → 768 B** (≥ `sizeof(MC_Params_t)` ≈ 668 B; the flash slot is
  2 KB, so there is ample room).
- **No `MC_PARAM_STORE_VERSION` bump** — and that is the point. `od_blob` is the **last** field of
  `MC_Params_t`, so a shorter old record maps exactly onto the front of the grown struct.
  `MC_PersistentStore_Read` now accepts `size >= stored`: it front-copies the stored bytes and
  **zero-fills the grown tail**. An old v3 record therefore still loads — `calib` (electrical/current
  offsets, mech/home zero) and every previously-saved OD entry survive the update. Only the entries
  that were *already being dropped* (position recall, notch) come up at their defaults and need one
  re-save. A version bump would instead reject the old record → full config wipe → loss of the
  alignment offset. Rejected for that reason.

### Recurrence guards (this bug has now bitten twice)

- **Compile-time:** `_Static_assert(sizeof(MC_Params_t) <= MC_PARAM_STORE_MAX_PAYLOAD)` — the struct
  can never silently outgrow the store payload.
- **Runtime:** `MC_Od_GatherPersistent` sets a truncation flag when it drops an entry; exposed via
  `MC_Od_PersistTruncated()` and mirrored to `g_mc_debug.store_blob_truncated`. A future blob
  overflow now shows up as a watchable flag instead of "a setting won't persist". **When it trips,
  raise `MC_PARAMS_OD_BLOB_MAX`** (never shrink the PERSIST set to fit).

## Migration (operator)

After flashing: existing gains + calibration are preserved. `position_recall_enable` (and the notch
settings, if used) come up OFF — **re-enable and save once**; from then on they persist. No other
action; no full re-tune, no re-alignment.

## Consequences

- Motor-internal persistence format only — **not** the shared contract; no `mc_if_od.h` /
  `MC_IF_PROTOCOL_VERSION` / CHANGELOG change.
- Growing the PERSIST set is now a one-line `MC_PARAMS_OD_BLOB_MAX` bump with automatic
  backward-compatibility, as long as `od_blob` stays the last field of `MC_Params_t`.
- Follow-up: surface `store_blob_truncated` in the GUI diagnostics so it is visible without a
  debugger; consider dropping the redundant PERSIST flag on `0x2500:1 est_electrical_offset_rad`
  (already stored in `calib`) to reclaim 8 B.
