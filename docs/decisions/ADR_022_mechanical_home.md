# ADR-022: Mechanical home — "set mechanical zero" (multi-turn position reference)

## Status

Accepted

## Date

2026-06-22

## Context

Position commands (`0x607A target_position`, PROFILE_POSITION) need to be relative to a
user-chosen **home**. The encoder config already carries a `mechanical_zero_offset_rad`, but that
is applied in the encoder decode (`mc_ssi_encoder.c`) as a **single-turn** offset (it wraps to
[0, 2π) immediately) — i.e. an encoder *mounting* alignment, not a position-command home. A home
for multi-turn position commands must zero the **absolute, continuous (multi-turn)** position the
state estimator produces. Separately, the `0x2700:1 cal_command` OD entry existed but was **dead**
(nothing consumed `g_od.cal_command`).

User decisions (asked): **multi-turn absolute home**, set by a **capture command** named *set mech
zero* (no new value entry; lightest contract change).

## Decision

- **Multi-turn home offset** `s_home_offset_rad` (scheduler): on capture, `s_home_offset_rad =
  s_est.mechanical.position_rad` (the current absolute continuous position). The OD
  `position_actual` (0x6064) is published **home-relative**: `position_actual = (mech_position −
  home) / POS_SCALE`. The estimator's absolute position is unchanged (still mirrored to
  `g_mc_debug.mech_position_rad`); `g_mc_debug.home_offset_rad` exposes the home for the watch
  window.
- **Trigger**: a new calibration command `MC_IF_CAL_SET_MECH_ZERO (3)` on `0x2700:1` — this also
  **wires the previously-dead `cal_command`**. The slow loop dispatches it to a
  `request_set_mech_zero` flag; the medium loop performs the capture (reads the live position) and
  requests a flash save. Also settable from the watch window via `g_mc_inject.request_set_mech_zero`.
- **Persistence**: the home is stored in `MC_CalibData_t` by repurposing the spare `reserved`
  (uint32) slot as `home_offset_rad` (float) — same 24-byte layout, and old flash (reserved = 0)
  reads back as `0.0f` (= no home), so it is binary-compatible (no store-version bump). Saved on
  capture (alongside the electrical/current calibration), reloaded + applied on boot.
- **Contract**: `MC_IF_CAL_SET_MECH_ZERO` added to `Interface/mc_if_od.h`; logged in
  `Interface/CHANGELOG.md` [3.1.0]. **No `MC_IF_PROTOCOL_VERSION` bump** — it is a new valid value
  for an existing entry, not a wire/OD layout change. The network MCU just forwards motor-owned
  `0x2700` SDO writes (no CMC code change); the PC tool adds a "set mechanical zero" action.

## Consequences

- The PC tool can SDO-write `0x2700:1 = 3` to home the axis; `0x6064`/`0x607A` are then relative
  to that point. `0x2700:2 cal_status` echoes the accepted command.
- The existing single-turn `mechanical_zero_offset_rad` (encoder mounting) is untouched and remains
  a separate concern.
- **D3 dependency**: `position_actual` is home-relative now; the PROFILE_POSITION *target* becomes
  relative once the position loop + trajectory (D3) land — they must drive against the same
  `mech_position − home`.

## Single-turn-encoder caveat (important)

Board profile 0's encoder is **single-turn absolute** (21-bit, one rev). A captured multi-turn home
is exact **within a power session**, but **across a power cycle** only the within-one-rev component
is recoverable — the estimator's turn count resets to 0 at boot, so a home captured several turns
out cannot be reconstructed from a single-turn reading. Practical consequence:

- Axes whose travel stays within ~1 rev: persistence is fully correct.
- Multi-turn axes: **re-home after power-up**. Robust multi-turn-across-power needs a homing
  routine (drive to an index/limit) or a multi-turn absolute encoder — future work. The capture
  itself is always correct in-session.

## Verification

- `gcc -fsyntax-only -std=c11 -I include -I ../Lightweight_CMC/Interface src/mc_scheduler.c
  src/mc_od.c src/mc_comms.c` → clean.
- **On-target (pending)**: with the drive idle, write `0x2700:1 = 3` (or set
  `request_set_mech_zero`), move the rotor, confirm `0x6064 position_actual` reads relative to the
  captured point and survives a save/reload (within one rev).

## Files affected

- include/mc_calib_data.h (`reserved` → `home_offset_rad`), include/mc_debug.h (`home_offset_rad`
  mirror + `request_set_mech_zero`), src/mc_scheduler.c (capture, apply, persist, `cal_command`
  dispatch)
- ../Lightweight_CMC/Interface/mc_if_od.h (`MC_IF_CAL_SET_MECH_ZERO`), ../Lightweight_CMC/Interface/CHANGELOG.md [3.1.0]
- docs/decisions/ADR_022_mechanical_home.md, docs/decisions/ADR_000_decision_log.md, docs/spec/05_object_dictionary.md

## Open questions

- Homing routine for robust multi-turn-across-power (re-establish the turn reference at boot).
- D3 position loop / trajectory to consume the home for PROFILE_POSITION targets.
- Whether to also expose the home value as a readable OD entry later (would be an OD-layout change → version bump).
