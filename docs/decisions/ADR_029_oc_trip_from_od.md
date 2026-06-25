# ADR-029: Wire the over-current trip threshold to the OD (current_trip_a)

## Status

Accepted

## Date

2026-06-24

## Context

The motor MCU's **only** fault source today is the measured over-current trip: `mc_scheduler.c` latches
`s_oc_trip` when `imax > g_mc_inject.current_limit_a`, where `imax = max(|ia|,|ib|,|ic|)` is the largest
absolute **measured** phase current and the threshold defaults to 3.0 A.

The OD entry **`current_trip_a` (0x2600:2, F32 RW PERSIST)** was *meant* to be the configurable trip,
but it was never applied: `od_mirror_live` did `g_od.current_trip_a = g_mc_inject.current_limit_a` every
medium tick, **overwriting** any GUI/OD write. So the trip was effectively pinned at the init default
(3.0 A) and only changeable via the SWD watch window. The user hit this while tuning the velocity loop —
a step pulse nuisance-tripped (measured-current overshoot past 3.0 A, with only 0.5 A of headroom over
the 2.5 A demand clamp `vel_current_limit_a`), and the trip couldn't be raised from the GUI.

## Decision

Make **`current_trip_a` (0x2600:2)** the source of truth for the over-current trip:

- `od_apply_gains` (slow loop — the safe OD→live apply point) sets the live trip from the OD:
  `g_mc_inject.current_limit_a = max(g_od.current_trip_a, 0.1f)`. The 0.1 A floor stops a stray
  0/negative from latching the trip permanently and locking the drive out.
- Removed the reverse mirror in `od_mirror_live` (it was clobbering the write).

The fast-loop trip and the alignment-current cap keep reading `g_mc_inject.current_limit_a`, which now
follows the OD value. **No contract change** (the entry already exists, RW PERSIST) and **no GUI change**
(the Motor Config tab already has the `0x2600:2` row). The trip is now GUI-settable and persists.

## Notes

- This is the **measured**-current trip. The velocity loop's **demanded** iq is separately *clamped* to
  `vel_current_limit_a` (0x2300:4) in the current-request generator — clamping is not a fault. Give the
  demand-clamp ↔ trip pair real headroom (e.g. demand 2.5 A, trip 4–5 A) so a tuning step's current-loop
  overshoot doesn't nuisance-trip.
- The same "OD-reflected but not OD-settable" pattern still affects the **observer gains**
  (`est_obs_*`, 0x2500:3-5), which are watch-window-sourced and mirrored to the OD. Not fixed here;
  noted for a follow-up if those need to be GUI-tunable.

## Verification

- `gcc -fsyntax-only` clean.
- On-target (user): write `current_trip_a` (0x2600:2) from the GUI Motor Config tab → read back sticks;
  the over-current trip fires at the new threshold; **Save to flash** persists it across reboot.

## Consequences

- Over-current trip is configurable from the GUI + survives a power cycle (PERSIST).
- Watch-window writes to `g_mc_inject.current_limit_a` are now transient (re-asserted from the OD each
  slow tick) — set the trip via the OD/GUI.

## Files affected

- `src/mc_scheduler.c` (`od_apply_gains` applies it + clamp; `od_mirror_live` no longer mirrors)
- `include/mc_debug.h` (comment), `docs/spec/12_faults.md`, ADR-029 + `ADR_000_decision_log.md`
