# ADR-019: Adopt Interface contract v2 (OD owner column + 0x3xxx CMC entries; protocol 1→2)

## Status

Accepted

## Date

2026-06-22

## Context

The network MCU extended the shared boundary contract to **v2** (`Interface/CHANGELOG.md`
[2.0.0], filed as `Interface/REQUESTS.md` REQ-0008). Three things changed:

1. Every entry in the `MC_IF_OD_OBJECTS(X)` X-macro grew a trailing **owner** column
   (`MC_IfOdOwner_t` = `MC_IF_OWNER_MOTOR` / `MC_IF_OWNER_CMC`).
2. A new **`0x3000-0x3033` block** of **CMC-owned** `axis_manager` entries was added (axis state,
   command triggers, op-mode + per-mode targets, limits). These are handled entirely on the
   network MCU; no SPI traffic is generated for them.
3. **`MC_IF_PROTOCOL_VERSION` bumped 1 → 2.** The wire packet layout (header, footer, all
   payload structs) is **byte-identical**; only the version byte and the OD-extension contract
   changed.

REQ-0008 briefed three motor-side actions: (1) add the `owner` param to every X-macro handler and
filter so CMC entries drop out of the build; (2) accept version 2 on incoming SPI frames; (3)
defensively return `NO_OBJECT` for any `0x3xxx` index that arrives over SPI.

## Decision

**Adopt v2 by rebuilding against the updated shared headers. No functional motor-side code change
is required.** Each briefed action is already satisfied — and the briefing's premise (that the
motor generates its OD from the X-macro) does not hold for this codebase:

1. **Owner filtering — not applicable.** The motor's OD table is **hand-maintained** — a static
   `s_od_table[]` walked by `MC_Od_Find` (`src/mc_od.c`). It does **not** expand
   `MC_IF_OD_OBJECTS(X)` (the only reference to that macro in the motor tree is a doc comment).
   So the new `owner` column is never seen by the motor build, and the `0x3xxx` CMC entries are
   simply absent from the hand table — no handler to change, no phantom entries to filter.
2. **Version — already via the shared constant.** `mc_comms.c` validates incoming frames against
   `MC_IF_PROTOCOL_VERSION` (`:195`) and stamps outgoing frames with it (`:43`), never a
   hard-coded `1`. A rebuild makes the motor accept/emit v2 and reject v1 with
   `MC_IF_ERR_BAD_VERSION` — exactly the required behaviour.
3. **Defensive `0x3xxx` → `NO_OBJECT` — already correct.** An index absent from the hand table
   resolves through `od_notfound()` → `MC_OD_ERR_NOT_FOUND` (`src/mc_od.c:190`), which
   `mc_comms.c:103` maps to the wire code `MC_IF_OD_ERR_NO_OBJECT`.

The only code touch is a clarifying comment in `src/mc_od.c` recording the owner model and that
the CMC-owned `0x3xxx` range is intentionally excluded.

## Reasoning

The motor OD has been a deliberately hand-maintained *subset* since ADR-015 (it carries only the
gains/telemetry/limits + CiA-402 objects the motor actually backs). That decoupling is why a
contract change which is "X-macro-signature breaking" on paper is a no-op here: the motor never
consumed the X-macro signature. Verifying against the code (not the cross-project briefing, which
assumed macro generation) is what surfaced this.

## Verification

- **Host build:** `gcc -fsyntax-only -std=c11 -I include -I ../Lightweight_CMC/Interface
  src/mc_comms.c src/mc_od.c` → clean (exit 0) against the v2 headers.
- **On-target (pending):** rebuild + flash v2; confirm (a) the v2↔v2 link runs (cyclic/telemetry
  counters climb, no `MC_IF_ERR_BAD_VERSION`), (b) an OD read of `0x3000` returns `NO_OBJECT`,
  (c) the motor-owned OD table behaves identically to v1.

## Consequences

- **Deploy is a coordinated cutover.** Because the motor accepts *only* `MC_IF_PROTOCOL_VERSION`,
  a v2 motor and a still-v1 CMC reject every one of each other's frames (`BAD_VERSION`) — the link
  is **down** in the gap. This is loud, not silent (no garbled data), but both MCUs must be flashed
  v2 close together. Order is immaterial (either mismatch breaks the link); the CMC author asked to
  flash the motor v2 first, then the CMC.
- **Standing drift risk.** The hand-maintained table is *not* compile-time-linked to the X-macro,
  so the shared canonical map and the motor table can diverge silently. v2 added nothing to the
  motor-owned ranges, so there is no divergence now — but the durable fix is to generate the motor
  table from `MC_IF_OD_OBJECTS(X)` with an owner filter (the X-macro/"unified config registry"
  refactor, REQ-0004 territory), still deferred.
- No change to timing domains, control loops, or the SPI transport.

## Files affected

- docs/decisions/ADR_019_interface_v2_adoption.md, docs/decisions/ADR_000_decision_log.md
- src/mc_od.c (clarifying comment only — no functional change)
- docs/spec/05_object_dictionary.md
- ../Lightweight_CMC/Interface/REQUESTS.md (REQ-0008 motor-side resolution)

## Open questions

- ~~Whether to close the drift risk now by moving the motor table to owner-filtered X-macro
  generation~~ — **resolved by ADR-020**: the motor OD table is now generated from
  `MC_IF_OD_OBJECTS(X)` (owner-filtered), so drift is a compile error. `0x2A00` into the OD
  (REQ-0004) remains the one deferred special-case.
