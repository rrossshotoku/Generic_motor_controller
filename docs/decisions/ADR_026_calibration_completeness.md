# ADR-026: Calibration completeness reporting + wire the current-offset cal command

## Status

Accepted

## Date

2026-06-22

## Context

The PC tool's new Motor Config tab (CMC repo `Interface/gui`) fires the OD calibration commands, but
the operator had no way to see **which calibrations are still outstanding** — only the transient
`cal_status` (0x2700:2), which just echoes the last command / running / fault and says nothing about
what has been done.

Investigation (trust-the-code) showed there was **no per-calibration completeness signal in the OD**:
`home_offset_rad` ("0 = none", ADR-022), `electrical_offset_rad`, and the current offsets all live in
`MC_CalibData_t` / runtime state and are **not OD-exposed**, so the GUI could not derive an
"outstanding" list. Separately, the contract's `MC_IF_CAL_CURRENT_OFFSET (2)` cal command was
**defined but never dispatched** by the scheduler, even though the current-offset routine itself
(`MC_CurrentSense_CalibrateOffsets`, run via `g_mc_inject.request_offset_cal` in the fast loop with the
power stage in safe-off) already exists.

User direction (asked): report completeness via a **new read-only OD bitfield**, tracking **all three**
calibrations — electrical alignment, mechanical zero, current offset — with the GUI listing the
*outstanding* (not-done) ones.

## Decision

**1. New OD object `0x2700:5 cal_done_flags`** (U16, RO, `MC_IF_F_NONE`, `MC_IF_OWNER_MOTOR`) — a
bitfield, set per calibration when that calibration has **valid data**; a clear bit = outstanding:

| bit | mask | calibration |
|----|------|-------------|
| 0 | `MC_IF_CAL_DONE_ELECTRICAL` (0x0001) | electrical-angle offset captured (alignment) |
| 1 | `MC_IF_CAL_DONE_MECH_ZERO` (0x0002) | mechanical home set (`0x2700:1 = 3`) |
| 2 | `MC_IF_CAL_DONE_CURRENT_OFFSET` (0x0004) | phase-current ADC offsets measured (`0x2700:1 = 2`) |

**2. Wire `MC_IF_CAL_CURRENT_OFFSET (2)`** on `0x2700:1`: the slow loop sets
`g_mc_inject.request_offset_cal` (the existing fast-loop routine then averages 2000 zero-current
samples and sets `s_cs.calibrated`). **Safety-gated**: rejected (`cal_status = FAULT`) unless the power
stage is off (`!s_pwm_on`), since the routine assumes no phase current is flowing.

**3. Completeness is DERIVED from existing state — no new persisted fields, no store-version bump**
(so previously-saved gains survive this update, which the user specifically relies on):
- electrical ⟺ `s_est_cfg.electrical_offset_rad != 0.0f` (default 0; an alignment result is
  `wrap2π(−pos·pp)`, essentially never exactly 0),
- mech-zero ⟺ `s_home_offset_rad != 0.0f` (ADR-022's existing "0 = none" convention),
- current-offset ⟺ `s_cs.calibrated`. To stop a flash *restore* from falsely reporting it as done,
  `params_save` writes a **0 sentinel** for the current offsets when `!s_cs.calibrated`, and boot
  restore only sets `s_cs.calibrated`/applies the offsets when the saved value is non-zero. (Old v2
  saves with real ≈2122-count offsets still restore as calibrated — acceptable; self-heals on next save.)

`od_mirror_live()` recomputes `g_od.cal_done_flags` from those three every medium tick (alongside the
existing `store_status` mirror).

## Alternatives considered

- **Persisted `cal_valid_flags` in `MC_CalibData_t`** — most explicit, but adding a field changes the
  flash payload layout → `MC_PARAM_STORE_VERSION` 2→3 → existing saved gains wiped on update (a
  regression the user would feel, having just got persistence working). A v2→v3 migration would avoid
  the loss but adds fragile flash-parsing code. Rejected in favour of the derive approach.
- **GUI-only heuristic** (no firmware change) — can't see mech-zero/current-offset (not OD-exposed), so
  the outstanding list would be incomplete. Rejected.

## Verification

- GUI smoke test (offscreen) extended: parses `0x2700:5`, decodes the three bits, renders the
  outstanding list; PASS.
- Firmware: host syntax check of the touched logic; on-target check is the user's bring-up step —
  read `0x2700:5` should show all three outstanding on a virgin store, then each bit set as alignment /
  set-mech-zero / current-offset run, surviving a power cycle once saved.

## Consequences

- The operator sees exactly what is left to calibrate; the GUI emphasises the outstanding ones and now
  has a **Current offset** action button (the command is finally wired).
- No flash format change → no loss of saved gains/calibration.
- Minor edge cases (documented): a legitimately-zero electrical offset or a home captured at exactly
  multi-turn 0.0 would read as "outstanding"; both are negligibly unlikely and consistent with the
  existing "0 = none" conventions.

## Contract impact (governance)

Additive RO OD entry + bit `#define`s in `mc_if_od.h`; **no wire/PDO-layout change → `MC_IF_PROTOCOL_VERSION`
stays 3**. Logged as **CHANGELOG [3.5.0]**. Consumers: motor MCU implements; **CMC forwards unchanged**
(motor-owned, generic OD bridge); **PC tool** reads + displays.

## Files affected

- `../Lightweight_CMC/Interface/mc_if_od.h` (0x2700:5 + `MC_IF_CAL_DONE_*` + current-offset-now-wired note)
- `../Lightweight_CMC/Interface/CHANGELOG.md` ([3.5.0])
- `include/mc_od_store.h` (`cal_done_flags` field)
- `src/mc_scheduler.c` (cal_command==2 dispatch + gate; current-offset save sentinel + restore tie;
  `od_mirror_live` completeness mirror)
- `docs/spec/13_calibration.md`, `requirements.yaml`
- `../Lightweight_CMC/Interface/gui/mc_gui/main_window.py`, `.../gui/smoke_test.py` (display)

## Open questions

- If single-turn home recovery proves fragile across reboot (ADR-022 caveat), mech-zero completeness
  may want an explicit persisted flag in a future store-version bump (deferred — avoided here to keep
  saved gains).
