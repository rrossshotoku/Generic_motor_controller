# Feedback and Encoders

## Position feedback abstraction

The control framework consumes common position sensor samples and mechanical/electrical state structs. It must not depend on SSI directly.

Initial backend:

- Configurable SSI absolute encoder over SPI-like peripheral

Future backend:

- Quadrature encoder using STM32 timer encoder mode

Both feed the same state estimator.

## SSI configuration

The SSI backend shall support:

- total frame bits
- position bit count
- position LSB location
- optional error bit
- optional warning bit
- optional parity
- odd/even parity selection
- counts per revolution
- mechanical zero offset
- direction inversion
- sample period

## State estimator

Outputs:

- mechanical position rad
- mechanical velocity rad/s
- mechanical acceleration rad/s^2 if implemented
- validity flags
- electrical angle rad for BLDC/FOC
- electrical velocity rad/s if useful

Electrical angle default:

```text
electrical_angle = wrap(mechanical_angle * pole_pairs + electrical_offset)
```

The estimator must handle wraparound and timestamping. Velocity may be finite difference with low-pass filtering initially.

## Realized (implementation, see ADR-008)

- **SSI read** (`mc_ssi_encoder_stm32g474.c`): SPI1 master, 16-bit, CPOL=1/CPHA=0 (Mode 2),
  MSB-first, ~1.33 MHz; two 16-bit words form a 32-bit frame; **position = bits [30:10]**
  (21-bit, binary). Blocking read in the 1 kHz medium loop.
- **Decode** (`mc_ssi_encoder.c`): extracts the position field and applies direction
  (board: invert) + mechanical zero offset -> single-turn mechanical angle.
- **Estimator** (`mc_state_estimator.c`): multi-turn continuous position via wrap detection;
  mechanical velocity by finite-difference + first-order low-pass (~20 Hz); electrical angle
  `wrap(single * pole_pairs + offset)`. Acceleration not yet estimated.
- **Velocity observer** (ADR-003 default) deferred to B2b; finite-difference is the current path.
- Board profile 0: AMM5B, 21-bit, counts/rev 2,097,152, position LSB 10, direction invert,
  pole pairs 11.
