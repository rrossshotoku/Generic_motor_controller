# ADR-049: Direct brushed current-loop gains (replace the bandwidth derivation)

- Status: Accepted
- Date: 2026-06-29
- Related: ADR-039 (brushed-DC backend), ADR-011 (PID)

## Context

ADR-039 set the brushed armature-current PI gains by *derivation*: the operator set a bandwidth
(`0x2400:8 hb_cur_bandwidth`) and the firmware computed `kp = wc·L`, `ki = wc·R` (pole-zero
cancellation), exposing `kp`/`ki` read-only at `0x2400:6/7`. That needs accurate `R`/`L`. During
brushed bring-up the operator wants to **hand-tune `kp`/`ki` directly** and watch the step
response, rather than trust a derivation off not-yet-identified `R`/`L`.

## Decision

Make the brushed current PI gains directly settable:

- **`0x2400:6 hb_cur_kp`, `0x2400:7 hb_cur_ki`: RO → RW, PERSIST.** The scheduler applies them
  verbatim (`s_hb_ipi_cfg.kp/ki = g_od.hb_cur_kp/ki`).
- **`0x2400:8 hb_cur_bandwidth`: removed**, along with the `R`/`L`→bandwidth derivation.

`R`/`L` (`0x2000:3/4`) still feed the motor model for other uses, but no longer drive the current
gains. Default `5.55 / 6300` (the ADR-039 bring-up bootstrap). GUI: the "Current loop gains
(0x2400)" group exposes `hb_cur_kp`/`ki` as editable fields (the bandwidth field is gone); the
brushed bandwidth estimate is now `hb_cur_kp / L`.

## Consequences

- **Contract change:** `0x2400:6/7` access RO→RW + PERSIST; `0x2400:8` removed. Non-PDO, no
  wire-format change → **no `MC_IF_PROTOCOL_VERSION` bump**; CMC unaffected (transport). PC tool
  re-syncs (bandwidth field gone, `kp`/`ki` now editable). Announced + logged in CHANGELOG.
- Old persisted blobs containing `0x2400:8` restore harmlessly: `MC_Od_RestorePersistent` writes
  each record via `MC_Od_Write`, which returns `NO_OBJECT` for the now-missing key and is
  `(void)`-ignored, so the rest of the config still restores; `hb_cur_kp/ki` default until set.
- Trade-off: the gains no longer auto-scale with `R`/`L` — the operator owns `kp`/`ki`. The
  derivation could return later as an optional "compute from bandwidth" helper if wanted.
