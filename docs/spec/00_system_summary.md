# System Summary

This framework is for a single reusable motor-axis module running on an STM32G474RET3 motor-control MCU. The system also includes a network MCU. Both MCUs use a real CiA 402-style object-dictionary model with index/subindex access.

## MCU split

```text
External network client
    -> network protocol layer, not fixed yet
    -> Network MCU CiA 402-style object dictionary
    -> SPI inter-MCU link
       - object read/write access
       - cyclic process-data exchange
    -> Motor-control MCU internal object dictionary
    -> Mode manager and motor-axis framework
```

The network MCU exposes the external communications interface. The motor-control MCU owns the real-time motor control.

## Initial motor implementation

- Motor type: BLDC/PMSM
- Control: three-shunt FOC
- Current sampling: PWM-triggered ADC, STM32G474-oriented
- PWM: TIM1 or TIM8 advanced timer, centre-aligned 3-phase PWM
- Position feedback: configurable SSI absolute encoder
- Electrical angle: derived from same encoder using pole-pair count and electrical offset

## Future expansion

A future brushed DC motor backend should be possible. Therefore:

- Outer motion framework must not depend on FOC-specific concepts.
- `id`, `iq`, electrical angle, Park/Clarke, and SVPWM stay inside the BLDC/FOC backend.
- Generic outer interface is torque/current request, not directly `iq`.
- Encoder feedback is abstracted so SSI and quadrature backends can both feed the state estimator.
