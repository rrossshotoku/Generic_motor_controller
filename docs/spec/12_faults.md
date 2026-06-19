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
