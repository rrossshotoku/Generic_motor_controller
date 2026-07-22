# ADR-072: Move idle/holding policy to the CMC — remove the motor's autonomous release (REQ-0016)

- Status: Accepted (motor side implemented; end-to-end acceptance pending on-target)
- Date: 2026-07-22
- Supersedes: the release behaviour of ADR-054 (holding-current release). Related: REQ-0016,
  ADR-021 (mode manager / controlword), ADR-069 (current demand limit), the brushed low-level
  current-feedback finding.

## Context

ADR-054 gave the motor an **autonomous** "release holding current ~1 s after the axis settles at
zero velocity" behaviour, gated by `0x2300:9 holding_enable = 0`. Diagnosing a drift/creep report
showed why this is awkward: when "released", the current loop keeps running with a zero setpoint,
hunting on near-zero feedback and dithering voltage into the motor (see the brushed current-feedback
finding). More fundamentally, *when to stop holding* is an application/axis decision, and splitting
it across two owners (the motor's dwell timer **and** the CMC's op arbiter) is implicit, hard-to-test
state. This is the mechanism-vs-policy split flagged in the ADR-068 direction: the motor should be a
servo that holds what it is told; the axis layer decides idle behaviour.

The CMC has taken ownership via a new CMC-owned entry **`0x3044 axis_holding_enable`**. On every
op-family release (JOYSTICK / SHOT_RECALL / HOMING → NONE) the CMC now explicitly sets:

- `0x3044 = 1` (default) → `op_mode = HOLD` — the motor keeps regulating to zero velocity.
- `0x3044 = 0` → `op_mode = OFF` — the CMC **disables the drive** (via the controlword) for
  high-stiction / back-drivable actuators where a sub-sensor-threshold holding voltage causes drift.

## Decision (motor side)

Remove the autonomous release logic entirely. The motor **always actively holds while enabled**, and
the drive is disabled only when the CMC commands it (CiA-402 controlword → the mode manager drops the
enable → `vel_active` goes false → the fast loop safe-offs the bridge). A true disable is a clean
off — no current-loop hunting, unlike the old "regulate to zero" release.

- Deleted from `mc_scheduler.c`: the `holding_enable == 0` dwell/settle release block, the
  `s_hold_released` / `s_hold_settle_ticks` state, the `MC_HOLD_RELEASE_TICKS` / `MC_HOLD_SETTLED_EPS`
  constants, and the entry-reset of that state. The velocity cascade now always runs the normal
  controller when active.
- **`0x2300:9 holding_enable` is kept but advisory only** (REQ-0016 pt 2): still readable/writable
  and PERSIST, but the motor never consults it. Default stays 1. Because the logic is *gone*, a
  leftover PERSIST `0` from older firmware is now harmless — strictly safer than the CMC's fallback
  suggestion of forcing the default to 1 (the value simply has no effect).
- Full removal of the OD entry is deferred to the next wire-breaking version (REQ-0016 pt 3) — it
  needs `MC_IF_PROTOCOL_VERSION` coordination with the CMC + PC tool.

## Consequences

- Single source of truth for idle behaviour (the CMC's op arbiter). The motor is a pure servo.
- The confusing "released-but-hunting" state is gone: idle is either an active hold (HOLD) or a clean
  drive disable (OFF) — the latter resolves the drift/dither symptom for high-stiction axes.
- **Fail-safe unchanged and correct:** on a lost link the `command_counter` dead-man still zeroes the
  demand, and the motor holds (does not autonomously drop the drive). Holding on failure is the safe
  default.
- No wire-format change (advisory entry retained) → no `MC_IF_PROTOCOL_VERSION` bump. Contract
  comment on `0x2300:9` updated to DEPRECATED/advisory; the CMC-owned `0x3044` is the live control.
- **Acceptance is on-target with both firmwares** (REQ-0016): `0x3044=1` → jog+release holds with
  current; `0x3044=0` → drive disables ~200 ms after the stick centres (statusword RUNNING →
  READY/DISABLED); `0x2300:9` proven irrelevant to behaviour. Not runnable here (needs the CMC + rig).
- Note: "always hold" holds *whatever mode you're in* — velocity mode holds zero **speed**, not
  position (a back-drivable axis still drifts under HOLD). Position hold requires position mode /
  `jog_position_mode` — orthogonal to this change (see the position-deadband ADR-071).
