# ADR-036: store_status bitfield + save-pending visibility

- **Status:** Accepted
- **Date:** 2026-06-25
- **Related:** ADR-010 (persistence — flash writes only with the power stage off), ADR-023 (PERSIST OD serialization)

## Context

A save (write `MC_IF_SAVE_MAGIC` to `0x2800:1`) gathers the PERSIST entries and **latches** the flash
write, but the write only commits while the **power stage is OFF** (ADR-010 — a flash erase/program stalls
the CPU for ~20–40 ms, which would freeze the 20 kHz FOC loop *and* the over-current trip). If the drive is
**enabled** when the user saves — and via the CMC it usually is, since `axis_manager` streams `ENABLE` —
the save sits latched and never commits; on reboot the value reverts to defaults. Nothing reported this:
`store_status` (0x2800:2) only reported `HasValid` (0/1), and the real `store_save_pending` flag was
watch-window-only. So saves were **silently lost** (the reported bug: set `0x6081`, save, reboot → default).

## Decision

Make **`store_status` (0x2800:2, RO) a bitfield** so a host can see a latched-but-uncommitted save:

- `MC_IF_STORE_VALID   (0x0001)` — a valid saved record exists in flash (the old `== 1` meaning, now bit 0).
- `MC_IF_STORE_PENDING (0x0002)` — a save is latched, awaiting power-stage-off to commit.

Motor: `store_status = VALID·HasValid() | PENDING·SavePending()` (one line in `od_mirror_live`).
PC tool: the Motor Config **"Save to flash"** polls `store_status` and shows **"SAVE PENDING — disable the
drive to commit"** while the pending bit is set, then **"Saved to flash"** once it clears (committed).
Backward-compatible — bit 0 preserves the "valid" meaning; consumers test `& MC_IF_STORE_VALID`, not `== 1`.

**Rejected: removing the power-stage-off gate.** A flash erase with the drive live freezes the FOC current
loop and the over-current trip for ~20–40 ms → loss of current regulation + protection → over-current /
hardware-damage / axis-jerk risk. The gate stays; the fix is **visibility**, not removing the safety.

Additive defines, non-wire → **no `MC_IF_PROTOCOL_VERSION` bump**. CHANGELOG [4.2.0].

## Consequences

- Saves are no longer silently lost: the user is told to disable the drive, and gets a confirmation when
  the write actually commits.
- `store_status` is now a bitfield; any other reader must test `& MC_IF_STORE_VALID` rather than `== 1`.

## Files

`../Lightweight_CMC/Interface/mc_if_od.h` + `CHANGELOG.md` [4.2.0]; `src/mc_scheduler.c`;
`gui/mc_gui/main_window.py`; `ADR_000_decision_log.md`.
