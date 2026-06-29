# ADR-048: Current-command notch filter for resonance suppression

- Status: Accepted
- Date: 2026-06-28
- Related: ADR-047 (resonance sweep — finds the bands to notch), ADR-011 (PID primitive)

## Context

The frequency sweep (ADR-047) reveals mechanical resonances (e.g. ~40–70 Hz). When the velocity
loop closes around such a resonance it commands torque at the resonant frequency, exciting and
sustaining it. We want a configurable band-reject to stop the loop driving that band.

## Decision

A 2nd-order IIR notch (RBJ biquad, `mc_notch.c/.h`) on the **velocity-loop current command**
(`s_iq_cmd_published` — the iq reference into the FOC current loop), run at the **1 kHz medium
rate**.

**Not** on the current controller's *voltage* output: a notch *inside* the ~250 Hz current loop
would be compensated (and the integrator wound up) by the loop, so it wouldn't actually suppress
the resonance. The effective place is the **command/reference** — the fast current loop then
tracks the already-notched command, so no resonant torque is produced.

1 kHz (not the 20 kHz fast loop) is chosen for numerical conditioning: a 55 Hz notch at 20 kHz
puts the biquad poles extremely close to `z = 1`; at the iq command's native 1 kHz rate it is
well-conditioned.

OD block `0x2930` (motor-owned, **PERSIST**): `notch_enable` (U8), `notch_freq_hz` (centre),
`notch_bandwidth_hz` (−3 dB width). Coefficients are recomputed (trig) only when the band
changes, off the control path; the biquad `Update` is trig-free. The notch **resets on
velocity-loop (re)start** (bumpless), and **torque mode + the sweep injection bypass it**.
Default: **off**, 55 Hz / 30 Hz (covers ~40–70 Hz). GUI: a "Notch filter — current command
(0x2930)" config group in Motor Config.

## Consequences

- A notch adds phase lag around its band — **verify velocity-loop stability with it enabled**
  (it ships off; enable it deliberately). Out-of-range params → passthrough (safe no-op).
- Additive PERSIST OD: no `MC_IF_PROTOCOL_VERSION` bump and no flash wipe (new entries default
  until set + saved); CMC unaffected (transport). Logged in CHANGELOG.
- Validated by `tests/test_notch.c` (deep null at f0, ~−3 dB at the band edges, flat passband).
- One biquad = one band. A second notch (or higher order) is a straightforward follow-up if more
  than one resonance needs rejecting.
