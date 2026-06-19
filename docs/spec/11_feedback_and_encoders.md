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
