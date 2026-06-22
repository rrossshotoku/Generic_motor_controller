# ADR-024: OD-triggered electrical alignment (current-regulated open-loop)

## Status

Accepted

## Date

2026-06-22

## Context

Electrical alignment (finding the encoder→rotor-magnet offset) was two manual watch-window steps:
drive an open-loop d-axis **voltage** (`align_voltage_v`) to pull the rotor to electrical 0, then
trigger `request_align_capture` (which just reads the position and computes
`electrical_offset = -position × pole_pairs`). The user wants it as a single OD command: enter the
alignment **current** + hold time, then trigger.

Design discussion: the *normal* FOC loop can't be used during alignment because it consumes the
estimator electrical angle — which is exactly what alignment produces. (The current loop *could*
run with the angle **forced** to 0, so it isn't strictly chicken-and-egg — but open-loop is the
better call regardless: alignment is a commissioning step that often happens *before* the current
loop is tuned, so it shouldn't depend on the d/q PI being stable.) **User chose current-regulated
open-loop** — enter a current, with the angle forced and a simple 1-D regulator, not the FOC loop.

## Decision

- **OD entries** (`mc_if_od.h`): `0x2700:3 cal_align_current_a` (F32, RW PERSIST) and
  `0x2700:4 cal_align_hold_ms` (U16, RW PERSIST). Persistent → carried in the ADR-023 gain blob.
- **Trigger**: write `0x2700:1 = MC_IF_CAL_ALIGN_CAPTURE` (value `1`, already in the contract) —
  the slow loop sets `request_align_routine`; the watch window can set it directly too.
- **Routine** (medium-loop state machine; *owns the command path* while running): force the
  electrical angle to 0; a slow integrator drives the open-loop d-axis voltage
  `Vd += KI·(i_target − i_d)` (where `i_d` = phase-A current at the forced angle 0, and `i_target`
  is `cal_align_current_a` clamped to 0.9·`current_trip_a`); after `cal_align_hold_ms` it captures
  `electrical_offset = wrap2π(−position·pole_pairs)`, goes safe-off, and `params_save()`s. It
  **aborts to safe-off on over-current**, and **starts only when the drive is idle** (otherwise
  `cal_status` → fault). `0x2700:2 cal_status`: `ALIGN_CAPTURE` while running, `NONE` on success,
  `0xFFFF` on fault.
- No FOC dependency: the angle is forced and the current regulator is a 1-D `Vd` integrator
  (`MC_ALIGN_KI_V_PER_A`), not the d/q PI. The existing watch-window voltage-align + capture-only
  path (`request_align_capture`) is unchanged.

## Contract

Two **motor-owned** OD additions, not in the cyclic frame → `CHANGELOG.md` **[3.2.0]**, **no
`MC_IF_PROTOCOL_VERSION` bump** and no link break (consistent with the [3.1.0] cal-code precedent).
The network MCU just recompiles against the updated header (it forwards motor-owned `0x2700` SDO
writes — no code change); the PC tool exposes `0x2700:3/4` + a "run alignment" action.

## Safety

- Idle-gated start (rejects if the drive is active/busy).
- Over-current abort → immediate safe-off.
- Hold time hard-bounds the drive — it always terminates and safe-offs.
- `Vd` clamped to `MC_C2_VD_MAX`; the current target clamped under the trip.
- **Triggering drives current and snaps the rotor to the aligned position** — the axis must be free
  to jerk. (Documented for the operator; the over-current trip + current clamp bound it.)

## Verification

- `gcc -fsyntax-only` of `mc_scheduler.c` + `mc_od.c` + `mc_comms.c`: clean.
- OD table dump: **65 entries**, `0x2700:3` and `0x2700:4` present; persistent count 30; the
  persistent gather (233 B, < 256) includes them.
- **On-target (pending)**: with the drive idle, set `0x2700:3` (e.g. 1 A) + `0x2700:4` (e.g. 1500 ms),
  write `0x2700:1 = 1`, watch the rotor align and `cal_status` go `1 → 0`, then confirm `0x2500:1
  est_electrical_offset_rad` holds a sensible captured value and the closed FOC then runs.

## Files affected

- ../Lightweight_CMC/Interface/mc_if_od.h (`0x2700:3/4`), ../Lightweight_CMC/Interface/CHANGELOG.md [3.2.0]
- include/mc_od_store.h (fields), src/mc_od.c (defaults), include/mc_debug.h (`request_align_routine`)
- src/mc_scheduler.c (align state machine + regulator + slow-loop dispatch)
- docs/decisions/ADR_024_od_electrical_alignment.md, docs/decisions/ADR_000_decision_log.md, docs/spec/05_object_dictionary.md

## Open questions

- `MC_ALIGN_KI_V_PER_A` is a conservative `#define`; could become a board-config value if some
  motors need a faster/slower ramp. `cal_align_hold_ms` must exceed the ~0.3–0.5 s current ramp.
- Phase-order detection (Phase E) could fold into the same routine later.
