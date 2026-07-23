# ADR-076: ON_TARGET / TARGET_REACHED tolerance follows the position deadband

- Status: Accepted
- Date: 2026-07-23
- Related: ADR-033/REQ-0013 (movement_status), ADR-035 (HALT), ADR-056 (abandon-plan on jog),
  ADR-071 (position deadband), REQ-0001 (CiA-402 statusword)

## Context

`MC_IF_MOVE_ON_TARGET` (movement_status) and `MC_IF_SW_TARGET_REACHED` (statusword 0x6041 bit 10) are
set from `target_reached` in the position cascade. The condition is
`complete && at_cmd_target && |perr| < window`:

- `complete` — the trajectory plan finished.
- `at_cmd_target` — the hold is still the CMC's commanded target; set **false** the moment a joystick
  jog abandons the plan (velocity-mode jog leaves position mode; position-integrated jog drops into
  the no-active-plan branch, ADR-056). So a manual trim already de-asserts ON_TARGET — good.
- `window` — was a **fixed `MC_POS_TARGET_WINDOW_RAD = 0.01 rad`**.

Requirement (broadcast): ON_TARGET means "the actuator is *on the shot*" — at the last CMC positional
target — and drops the instant the joystick moves it off. The first two clauses already give that; the
issue is the fixed window. With the position deadband (ADR-071) active, the position loop applies no
correction within ±deadband, so the axis **parks anywhere in that band**. A fixed 0.01 rad window that
is smaller than the deadband would then report "not on target" even though the axis is parked exactly
where it is going to stay.

## Decision

Use the **position deadband** as the ON_TARGET / TARGET_REACHED window:

```
twin   = (position_deadband_rad > 0) ? position_deadband_rad : MC_POS_TARGET_WINDOW_RAD
reached = complete && at_cmd_target && |perr| < twin
```

So "on the shot" is reported over exactly the band the axis actually parks in. When the deadband is
**off (0)**, it falls back to the previous fixed 0.01 rad so the bit stays reachable (the loop drives
to the exact target, so a small non-zero window is still needed). Read via `s_pos_cfg.deadband_rad`
(the config already copied at the OD boundary in `od_apply_gains`), not `g_od` directly.

## Consequences

- ON_TARGET is now consistent with where the axis settles: with the deadband armed it asserts as soon
  as the axis is parked within the band, instead of demanding a tighter fixed window it may never meet.
- No new OD entry, no wire/layout change → **no `MC_IF_PROTOCOL_VERSION` bump**. But the *semantics* of
  a consumed status bit change (the tolerance now tracks `position_deadband_rad`), so it is logged in
  the Interface CHANGELOG (Changed) for the CMC/PC — no CMC code change required; it just reads the bit.
- `at_cmd_target` behaviour is unchanged: a joystick jog (either mode) still de-asserts ON_TARGET.
- **Edge note (unchanged, not in scope):** HALT hold (ADR-035) leaves `at_cmd_target = true`, so a HALT
  at a non-shot position can still report ON_TARGET within the window. If that matters, treat it
  separately — this ADR only retargets the tolerance.
- Verification build + review only. Bench: set `position_deadband_rad`, complete a shot, confirm
  ON_TARGET asserts within ±deadband; jog off with the stick, confirm it de-asserts; set deadband 0 and
  confirm the 0.01 fallback still asserts on a shot.
