# ADR-059: PC-tool config snapshot (save/load to file)

- Status: Accepted
- Date: 2026-07-02
- Related: ADR-010 (persistent store), the CMC axis_manager

## Context

There was no way to back a unit's tuned configuration up off-device, restore it after a firmware
reflash that wipes flash, or clone config to another unit — only the on-device flash save.

## Decision

GUI-only: two buttons on the CMC **Setup** tab.
- **"Save config → file…"**: reads every RW+PERSIST OD entry (motor `0x2xxx` + CMC `0x3xxx`) from the
  connected device and writes a JSON snapshot `{key: {name, raw}}` to a `.json` file on the PC. It
  saves the **on-wire raw value**, so it round-trips exactly regardless of scaling. Async reads are
  collected in `_on_read_done`; an 8 s backstop finalises with whatever came back.
- **"Load config ← file…"**: parses the JSON and `write_async()`es each entry's raw value to the device
  (RAM), skipping keys not writable / not in this build's OD. The operator then clicks **Save to flash**
  to commit.

## Consequences

- GUI-only — **no contract change, no `MC_IF_PROTOCOL_VERSION` bump, no motor change.** Lives in the
  Lightweight_CMC GUI.
- Round-trips the raw value, so it's robust to scaling and forward-compatible (unknown keys skipped
  with a count, rather than failing).
- Enables off-device backup, post-reflash restore, and unit-to-unit config cloning. Requires a
  connection for both save (reads live values) and load (writes them).
