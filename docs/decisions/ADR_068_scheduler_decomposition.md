# ADR-068: Scheduler decomposition — extract inlined state machines into modules (start: mc_homing)

- Status: Accepted (phases 1, 3, 4 done; phase 2 deferred — see below)
- Date: 2026-07-22
- Related: ADR-057 (homing), ADR-005 (timing domains / bring-up), ADR-019 (OD generation),
  ADR-065/066/067 (recent feature modules whose *orchestration* accreted into the scheduler)

## Context

`mc_scheduler.c` has grown to **1819 lines — 35 % of all source**, 5× the next-biggest module.
`MC_MotionLoop_1kHz` alone is ~790 lines. Meanwhile the rest of the tree is healthy: 33 of 34 `.c`
files are ≤340 lines and single-purpose, and the architecture's seams hold — HAL is confined to
`*_stm32g474.c` boundary modules, no controller reads the OD directly, FOC concepts stay in the
FOC backend. The problem is localised: every new feature ships a clean small module
(`mc_thermal`, `mc_dither`, `mc_pos_recall`) **plus** a slab of orchestration (edge detection,
settle dwells, gating, sequencing) inlined into the loop bodies. The modules stay clean; the
scheduler is where coupling accretes.

The goal end-state (see the project guide) is a scheduler that is a thin **dispatcher**: per timing
domain it reads inputs, calls `MC_X_Update/Service(...)` on self-contained modules, and applies
their outputs — with no embedded multi-state logic.

## Decision

Incrementally extract the inlined state machines into their own HAL-free, host-compilable modules,
one reviewable slice at a time, each behaviour-preserving. **Phase 1 extracts the homing
sequencer** (`mc_homing`) — the single largest self-contained lump, and the highest-value slice
because homing is safety logic that benefits most from being independently testable.

### What moves vs. what stays

The homing logic is entangled with scheduler-wide state, so the boundary is drawn deliberately:

- **Moves into `mc_homing` (sequencer-private):** the phase (idle / approach / back-off), the
  timers (`still` / `total` / `backoff`), the `moved` arming flag, the `MC_IF_HOME_*` status, and
  the `MC_HOME_*` timing constants. The module is a pure function of its inputs → a command struct.
- **Stays in `mc_scheduler` (shared by ~12 sites):** `s_home_offset_rad` (the mechanical-zero
  anchor read by position_actual, soft limits, recall, jog, tuning), `s_homed`, `s_oc_trip`, the
  velocity **slew limiter** (`vel_slew_*`, driven by OD accel params 0x2300:6/7/8), `params_save`,
  and the statusword. The module never touches these directly — it *signals* the scheduler to.

### Interface (behaviour-preserving)

```c
MC_Homing_Update(state, in, out);   // called once per medium tick
```
- `in`: `enable` (home_command≠0 && !inject && !dq_test), `clear` (home_command==0),
  mech position + velocity, `oc_trip`, `home_velocity_rad_s`.
- `out`: `status` (mirror to `g_od.home_status`), `active` (arbiter takes the homing branch),
  `want_drive`, `velocity_cmd` + `slew` (the scheduler applies `vel_slew_limit` when `slew`, else
  uses the raw value — preserving the exact one-tick `s_eff_vel_cmd = 0` at stop capture),
  `reset_slew` (scheduler calls `vel_slew_reset(current_vel)`), `capture_zero` (scheduler sets
  `s_home_offset_rad = position`), `consume_oc_trip`, `completed` (scheduler sets `s_homed` +
  `params_save`).

The five-way command arbiter (inject / dq-test / homing / remote) keeps its shape; the homing
branch shrinks to "set the mode defaults, apply `want_drive`/`velocity_cmd`, publish statusword".
Slew reset/limit ordering is preserved (reset in the pre-arbiter call, limit in the branch, same
tick). Because the module emits raw targets + explicit slew-control flags, the extraction is
byte-for-byte behaviour-preserving against the inlined version — verified by review + clean build.

## Verification & risk

- **No host or on-target test ran** this change: homing tests aren't in the suite and no hardware
  was available. Verification is a clean firmware build plus a line-by-line behaviour-equivalence
  review against the pre-extraction code. **Homing drives the axis into a hard end stop — bench-test
  it on target (approach → stop capture → back-off → DONE; abort; timeout → FAILED) before relying
  on it.** This is the reason a golden-reference homing test is now a tracked follow-up.
- Risk is contained to homing; the shared anchor state and slew limiter are untouched, so
  position_actual / soft limits / recall / jog behaviour cannot change.

## Consequences

- `mc_scheduler.c` drops ~65 lines and the medium loop's homing slab becomes a ~15-line call +
  a thin arbiter branch; homing logic is now unit-testable in isolation.
- Establishes the extraction pattern for subsequent slices.

## Phases 3 & 4 (2026-07-22) — slow-loop dispatcher

Done together as the low-risk pair (build + review verified; no motor motion in either).

- **Phase 4 — slow-loop OD-command dispatch.** The flat block that routes OD command words
  (0x2800 store magics, 0x2700:1 cal commands, 0x2910:6 test trigger) to latched requests moved
  **verbatim** into `static void od_commands_service_slow(void)`. Kept in-file, not a module: the
  handlers latch scheduler-private state (`s_set_zero_at_pending`, `g_mc_inject` requests), so a
  module would need a wide interface for no gain. The factory-reset / save wiring is unchanged.
- **Phase 3 — normalise the remaining slow-loop blocks** into sibling `*_service_slow()` helpers
  (`persistence_service_slow`, `thermal_service_slow`, `command_deadman_service_slow`) alongside the
  existing `pos_recall_service_slow`. `MC_SlowLoop_10_100Hz` is now a 7-line dispatcher (BootMeta →
  commands → persistence → recall → thermal → apply-gains → dead-man) with the **same ordering**
  (thermal derates the current limit before `od_apply_gains` pushes it). **Dither was left inline**:
  its logic already lives in `mc_dither`; the injection is a 3-line step *inside* the velocity
  cascade (after the notch, on the published iq) that can't move without threading the local iq
  in/out — extracting it would add noise, not clarity. Net: a few lines of wrapper for a loop body
  that reads as named services instead of ~110 lines of inline blocks (line count is not the metric
  here — organisation is; the reduction came in phase 1).

## Phase 2 — deferred

The electrical-alignment routine (`mc_align`) is the remaining substantive slice but is **not
low-risk**: it energises the motor and produces the `electrical_offset_rad` commutation datum, and
it shares the `s_eff_align*` command variables with the inject / dq-test paths. Deferred by the user
(2026-07-22) because the rotor is currently fully loaded, so the on-target alignment test needed to
validate the extraction can't be run. Revisit paired with a bench test (and ideally a host test).

## Follow-ups

- A **golden-reference host test for `mc_homing`** (now that it is a standalone HAL-free module):
  approach → stop capture → back-off → DONE; abort; timeout → FAILED. This is the verification the
  inlined code never had, and the harness the riskier phase-2 slice will want.
- Consider renaming `mc_boot_meta.c` to the `*_stm32g474` boundary convention (it uses HAL but
  doesn't carry the suffix).
