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
