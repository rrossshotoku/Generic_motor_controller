# ADR-066: Low-speed anti-stiction current dither

- Status: Accepted
- Date: 2026-07-17
- Related: ADR-042 (velocity demand), ADR-048 (current-command notch), ADR-065 (thermal), ADR-030 (tuning signal generator)

## Context

Near zero speed, static friction (stiction) dominates and the axis tends to stick then
jump (stick-slip), hurting low-speed smoothness and fine positioning. A small oscillating
("dither") current keeps the mechanism in the kinetic-friction regime so it responds
smoothly to small commands. It's only wanted at low speed — at speed it just adds loss,
noise and heat.

## Decision

Add a **low-speed current dither**: a zero-mean sine current added to the velocity/position
loop's current command, faded out as speed rises.

- **Injection:** the medium loop (1 kHz) adds it to `s_iq_cmd_published` **after the velocity
  loop and the notch filter** (so the notch can't cancel it), before the fast current loop
  consumes it. It then passes through the normal current limit, so it can't push past the
  limit or nuisance-trip the OC. Applies in velocity/position (closed-loop) driving only —
  not torque mode, not while disabled/hold-released.
- **Waveform:** zero-mean **sine** (`amplitude · sin(phase)`), phase accumulator advanced at
  `dither_freq_hz`. Zero-mean → no net velocity/torque bias.
- **Speed gate = smooth fade** (not hard on/off): `fade = clamp((threshold − |v|)/threshold,
  0, 1)`. Full at v=0, linearly to 0 at the threshold, off above. No chatter at the boundary,
  no torque step as the axis starts moving, and it self-disables at speed. Reuses the
  threshold knob — no hysteresis parameter needed.
- Chosen with the user: fade + sine + medium-loop generation (freq useful to ~200–300 Hz;
  higher would need fast-loop generation, not needed for stiction).

## OD (shared contract, additive — no version bump)

New block **`0x2320` — low-speed dither** (motor-owned, PERSIST):
`:1 dither_enable` (U8), `:2 dither_speed_threshold_rad_s` (F32), `:3 dither_amplitude_a`
(F32, A), `:4 dither_freq_hz` (F32), `:5 dither_output_a` (F32 RO **PDO** — the current being
injected now, graphable). GUI: a "Dither (0x2320)" group under Motor Config → Control loops.

## Runtime

`mc_dither` module (HAL-free): phase accumulator + speed fade + `sinf`. Cheap (one `sinf`
per medium tick). Off by default (`enable=0` → no injection) → zero behaviour change until
configured. The injected current is counted by the thermal model (small extra heating —
accounted for) and the OC trip (keep amplitude within the current-limit headroom).

## Consequences

- One clean seam (after the notch), one small module; the old current path is untouched when
  disabled. `dither_output_a` (PDO) lets you graph exactly what's injected while tuning it.
- **Follow-ups:** optional square/triangle waveform; fast-loop generation if >~300 Hz is ever
  needed. Golden-reference: at v=0 the injected current should be a clean zero-mean sine of
  `amplitude` at `freq`; it should fade to 0 by the threshold speed.
