# ADR-060: Mechanical-zero centering (midpoint of travel)

- Status: Accepted
- Date: 2026-07-03
- Related: ADR-022 (mech zero), ADR-038 (single-turn startup anchor / wrap seam)

## Context

The single-turn absolute (SSI) encoder loses its turn count at power-off; the startup anchor wraps the
reading to the nearest turn of the mech zero, so the cold-boot position always lands within ±half a turn
(±π) of the zero. If the axis parks near that ±π **seam**, tiny boot-to-boot differences flip it a full
turn — the position reads the wrong sign ("sometimes negative"). The seam is a hard 1-count threshold
(no hysteresis, and none can help at cold boot — there's no prior turn state to hold). The only robust
remedy for a **sub-1-turn** axis is to place the seam OUTSIDE the travel range — i.e. centre the mech
zero on the travel midpoint so each extreme sits at ±half-travel and the ±π seam falls in dead space
beyond the ends.

## Decision

- **Motor:** a new cal command **`MC_IF_CAL_SET_MECH_ZERO_AT`** (`0x2700:1 = 4`) sets `s_home_offset_rad`
  to the value in a new **`mech_zero_set_rad`** (`0x2700:10`, F32 RW, transient) — i.e. the home can be
  set to a *computed* position **without driving the axis there**. Auto-saved; also sets `s_homed`. The
  existing "capture current position" zero (cmd 3) is unchanged.
- **PC tool:** a **"Centre mech zero"** row on the Motor Config calibration page — *Capture CCW end* /
  *Capture CW end* read the absolute mechanical position (`0x2510:1`) at each travel extreme, and
  **Set zero = midpoint** writes `mech_zero_set_rad = (ccw + cw) / 2` then fires cal command 4.

## Consequences

- Additive OD (new cal-command code + `mech_zero_set_rad`, non-PDO) → no `MC_IF_PROTOCOL_VERSION` bump;
  the CMC forwards it. Not persisted (transient command input); persist budget unchanged.
- Fixes the ADR-038 seam for a sub-1-turn axis by construction (seam pushed to the travel ends). For a
  **multi-turn** axis the single-turn ambiguity remains (needs homing / a multi-turn encoder).
- `0x2510:1` is the same absolute frame the offset is captured from (the `position_actual = mech − offset`
  invariant), so the midpoint is exact and no axis movement is needed.
