# ADR-047: Stepped-sine current sweep for resonance / frequency-response identification

- Status: Accepted
- Date: 2026-06-28
- Related: ADR-030 (loop-tuning test-signal generator), ADR-018 (command arbitration)

## Context

To find mechanical/electrical resonances we need a frequency response (FRF / Bode): inject a
current sinusoid, sweep the frequency, and measure the velocity response. The existing
test-signal generator (ADR-030) produces ramps/steps in the **1 kHz** medium loop — a sinusoid
there is a coarse zero-order-hold staircase (~10 samples/cycle at 100 Hz, basically unusable
above ~150–200 Hz, and the source of the audible 1 kHz artifact the operator saw).

## Decision

A new stepped-sine sweep generator (`mc_freq_sweep.c/.h`) injected as the iq command in the
**20 kHz fast loop** (100 samples/cycle at 200 Hz — clean across the target band). At each
frequency from `start_hz` to `end_hz` (advancing by `step_hz`) it outputs
`bias + amplitude*sin(2*pi*f*t)` for `dwell_s`, then steps; **phase is continuous across steps**
(no click). The whole sweep — dwell timer, frequency step, phase — advances in the single
fast-loop `MC_FreqSweep_Sample` call, so there is no cross-loop race; the medium loop only
mirrors `current_hz`/`active` to the RO entries.

OD block **`0x2920`** (motor-owned, not persisted): `start/end/step_hz`, `dwell_s`, `bias_a`,
`amplitude_a`, `enable` (RW); `current_hz` (RO, PDO-mappable), `active` (RO). The fast loop
edge-detects `enable` (0→1) to Start and auto-stops past `end_hz`. Injected **only in
torque/current mode**; the velocity/current limit and OC trip still apply.

GUI: a "Frequency sweep (resonance ID)" section under the Tuning area — start/end/step/dwell/
bias/AC-amplitude fields, Start/Stop, and a live frequency readout. The operator graphs
`0x2920:8` (current frequency) against `0x2310:2` (actual velocity); resonances show as
response peaks.

## Consequences

- New OD block, **additive**; `0x2920:8` is PDO-*mappable* but not in the default cyclic frame,
  so no `MC_IF_PROTOCOL_VERSION` bump and the CMC is unaffected (it forwards these like any
  motor entry). The operator can add `0x2920:8` to the telemetry map (`0x2A00`) for high-rate
  graphing of the swept frequency.
- Reuses the FOC iq path; `sinf` at 20 kHz is cheap (the FOC already does sincos there).
- Validated by `tests/test_freq_sweep.c` (visits every frequency, stays within ±amplitude,
  ends past `end_hz`, `current_hz` never leaves range).
- **Bode capture (implemented, GUI-only):** a "Plot Bode at end" option records the streamed
  `tlm_vel_actual_rad_s` per frequency (binned by the sweep schedule from `t0` + dwell), detrends
  each dwell (removes DC + any bias-driven velocity ramp), takes RMS·√2, and **scales by `f` to
  normalise out the rigid-body 1/ω velocity rolloff** (`v = τ/(Jω)`) so resonances/anti-resonances
  stand out. On finish it pops a non-modal pyqtgraph frequency-vs-amplitude window. Needs
  `tlm_vel_actual_rad_s` streamed at cyclic rate ≥ 1 kHz; longer dwell = cleaner low-frequency points.
