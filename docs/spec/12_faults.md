# Fault Manager

## Fault severity model

| Severity | Behaviour |
|---|---|
| Warning | report and/or derate |
| Recoverable | controlled stop, inhibit new motion, latch, require reset |
| Severe | immediate PWM/current disable where necessary, latch, require reset |

## Codeable fault matrix fields

Each fault shall define:

- fault ID
- name
- detection condition
- threshold source
- debounce/filter time
- severity
- action
- latch behaviour
- reset condition
- object dictionary reporting

## Initial fault matrix

| ID | Fault | Detection | Severity | Action |
|---:|---|---|---|---|
| 1 | SPI timeout | no valid cyclic command for timeout | Recoverable | quick stop if valid, then inhibit |
| 2 | Encoder invalid | no valid position sample for N medium ticks | Severe | PWM/current off |
| 3 | Following error | abs(position error) > threshold for debounce | Recoverable | controlled stop, latch |
| 4 | Fast overcurrent | abs(current) > fast limit | Severe | immediate PWM off |
| 5 | Bus overvoltage | bus voltage > threshold | Severe or recoverable | inhibit/PWM off |
| 6 | Bus undervoltage | bus voltage < threshold | Recoverable | inhibit motion |
| 7 | Overtemperature warning | temp > derate threshold | Warning | derate current |
| 8 | Overtemperature fault | temp > trip threshold | Severe | PWM/current off |
| 9 | Soft limit reached | position beyond soft limit | Recoverable | stop away/inhibit direction |
| 10 | FOC voltage saturation | saturated for too long | Warning/recoverable | report or controlled stop |
| 11 | ADC invalid | current sample invalid | Severe | PWM/current off |
| 12 | Calibration failed | calibration routine failed | Recoverable | inhibit enable until reset |

Threshold values shall come from configuration/OD objects and may be persistent.

## Realized — implementation status

The fault **matrix is not yet implemented**. Today only **#4 Fast overcurrent** is detected and acted on;
`error_code` / `error_register` / `fault_flags` (0x2600:1) are still published as 0.

- **#4 Fast overcurrent** (`mc_scheduler.c`, fast loop): latches `s_oc_trip` when the largest absolute
  **measured** phase current `max(|ia|,|ib|,|ic|)` exceeds the threshold; severe action = block the FOC /
  hold the power stage safe; reset via the fault-reset controlword bit (clears `s_oc_trip`). It is the
  sole input to the mode manager's FAULT state (`fs.severe_active`) and the statusword FAULT bit.
  - **Threshold source (ADR-029):** the OD entry **`current_trip_a` (0x2600:2, RW PERSIST)**, applied to
    the live trip in `od_apply_gains` (floored at 0.1 A). GUI-settable + persists. Default 3.0 A.
  - This is the **measured**-current trip. The velocity loop's **demanded** iq is separately *clamped*
    to `vel_current_limit_a` (0x2300:4) by the current-request generator — a clamp, not a fault.
- **#1-3, #5-12 are not implemented.** Notably: bus over/under-voltage (#5/#6) need a Vbus sensor (none on
  board profile 0; the value is injected); over-temperature (#7/#8) need a temp sensor + the thermal model
  run for derating (not wired); following-error (#3), soft-limit (#9), encoder-invalid (#2), FOC-saturation
  (#10), ADC-invalid (#11), SPI-timeout (#1) detection are TODO. See ADR-029 and the project status notes.
