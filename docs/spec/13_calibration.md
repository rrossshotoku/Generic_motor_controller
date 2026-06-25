# Calibration Architecture

The initial framework shall include full calibration architecture with state-machine skeletons.

## Calibration routines

- Current sensor offset calibration
- Encoder alignment
- Phase order detection
- Motor electrical angle offset calibration
- Soft-limit learning
- Homing sequence
- Calibration storage

## Common calibration requirements

Each routine shall define:

- preconditions
- required mode/state
- safety gates
- state sequence
- applied current/velocity/position commands
- timeout
- abort conditions
- success criteria
- stored result
- validation checks

## Safety

Calibration is allowed only when:

- drive is in calibration/homing mode
- severe faults are not active
- user/OD has explicitly requested the routine
- output limits are reduced to calibration limits
- timeout is active

## Persistent results

Store selected results in the flash-backed parameter store:

- current offsets
- encoder zero offset
- electrical angle offset
- encoder direction
- phase order result
- soft limits
- homing reference

## Realized — completeness reporting + current-offset OD trigger (ADR-026)

A read-only OD bitfield **`0x2700:5 cal_done_flags`** (U16) reports which calibrations currently have
valid data, so a tool can show what is still **outstanding** (a clear bit = not done):

| bit / mask | calibration | "done" derived from |
|---|---|---|
| `MC_IF_CAL_DONE_ELECTRICAL` (0x0001) | electrical-angle offset (alignment) | `electrical_offset_rad != 0` |
| `MC_IF_CAL_DONE_MECH_ZERO` (0x0002) | mechanical home | `home_offset_rad != 0` (ADR-022) |
| `MC_IF_CAL_DONE_CURRENT_OFFSET` (0x0004) | phase-current ADC offsets | `s_cs.calibrated` |

Completeness is **derived from existing persisted/runtime state** (no new persisted fields, no
`MC_PARAM_STORE_VERSION` bump): `od_mirror_live` recomputes the flags every medium tick. For the
current offsets, `params_save` writes a **0 sentinel** when they were never measured, so a flash
restore cannot report current-offset calibration as done when it merely loaded board-nominal offsets.

The `MC_IF_CAL_CURRENT_OFFSET (2)` command on **`0x2700:1`** is now dispatched (previously inert): it
triggers the existing `MC_CurrentSense_CalibrateOffsets` routine (averaged zero-current samples in the
fast loop). **Safety gate:** accepted only with the power stage off (`!s_pwm_on`); otherwise
`cal_status = 0xFFFF` (fault), since the routine assumes no phase current is flowing.
