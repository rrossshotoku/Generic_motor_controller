# ADR-077: ON_TARGET / TARGET_REACHED survives the drive being disabled after a shot

- Status: Accepted
- Date: 2026-07-23
- Related: ADR-033/REQ-0013 (movement_status), ADR-056 (abandon-plan on jog), ADR-071 (deadband),
  ADR-072/REQ-0016 (idle policy → CMC; OFF disables the drive), ADR-076 (ON_TARGET tolerance)

## Context

Diagnosed live on the brushed incremental actuator (192.1.0.101) after a shot recall: it sat 0.0157
rad from the target — **well inside** its 0.1 rad deadband — yet ON_TARGET never asserted. The OD
showed `statusword = 0x1` (READY, not ENABLED), `modes_of_operation = 0`, and CMC
`axis_holding_enable = 0` (**OFF**). So after the shot the CMC (correctly, for this high-stiction axis
that drifts if energised) transitioned op_mode → OFF and **disabled the drive**.

`target_reached` (→ `MC_IF_MOVE_ON_TARGET` and statusword `TARGET_REACHED`) was only ever computed
**inside the position cascade**, which runs only while the drive is enabled and in position mode
(`if (s_eff_drive && s_eff_position_mode …)`). The moment the drive disabled, the code took the else
branch and **forced `target_reached = false`** — reporting "not on the shot" while parked exactly on
it. The brushless axis reports correctly only because it uses HOLD (`axis_holding_enable = 1`) and
stays enabled. So the split was HOLD-vs-OFF, not absolute-vs-incremental.

## Decision

Make ON_TARGET reflect "parked at the last CMC target," independent of whether the drive is currently
enabled. Latch it and hold it across a drive-disable:

- **Cascade (drive enabled + position mode):** unchanged `reached = complete && at_cmd_target &&
  |perr| < twin`; additionally, when `reached`, capture the shot position `s_shot_pos_rad = p_dem`.
- **Else branch (cascade not running):**
  - **Drive disabled** (`!s_eff_drive`, e.g. the OFF idle policy parked us on the shot and dropped
    the drive): **keep** `target_reached` latched, but drop it if the axis is **back-driven** beyond
    the deadband of `s_shot_pos_rad` (it's no longer on the shot).
  - **Drive enabled but out of position mode** (a velocity-mode joystick jog): **clear** it — moved
    off the shot. (A position-integrated jog already clears it via `at_cmd_target` in the cascade.)
- **Publishing:** `od_mirror_live` sets *both* `MC_IF_MOVE_ON_TARGET` and statusword
  `MC_IF_SW_TARGET_REACHED` from the latched `target_reached`, so the two agree in every state
  (enabled, disabled, jogged). The cascade no longer sets the statusword bit directly — moved to the
  one place, which also fixes the disabled case (statusword is freshly assigned by the arbiter each
  cycle, so the OR can't leave a stale bit).

A **new move** clears it naturally: the CMC re-enables the drive, the cascade runs, and `reached` is
false during the move.

## Consequences

- An OFF-policy axis (high-stiction / back-drivable) now reports on-shot correctly while parked and
  de-energised — the primary fix. HOLD-policy axes are unchanged.
- ON_TARGET now means "at the last commanded shot and hasn't been moved off," which is the stated
  requirement; it drops on a joystick jog, a new move, or being back-driven out of the deadband.
- Motor-internal — no new OD entry / no wire or layout change → **no `MC_IF_PROTOCOL_VERSION` bump**.
  But the *observable behaviour* of two consumed bits changes (they persist through disable), so it is
  logged in the Interface CHANGELOG (Changed). No CMC/PC code change required.
- `TARGET_REACHED` is now asserted in the CiA-402 "ready" (disabled) state when parked on the shot —
  a deliberate extension of the standard for this application (movement_status ON_TARGET is the
  primary indicator; the statusword bit is kept consistent with it).
- Verification is build + review only. On-target (192.1.0.101): re-run the shot recall after flashing
  and confirm ON_TARGET / TARGET_REACHED assert while the drive sits disabled on the shot, drop on a
  joystick jog, and re-clear during the next move.
