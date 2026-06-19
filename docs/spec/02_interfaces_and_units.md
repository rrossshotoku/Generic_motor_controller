# Interfaces and Units

## Internal units

Use SI/physical units internally:

| Quantity | Unit |
|---|---|
| Position | rad |
| Velocity | rad/s |
| Acceleration | rad/s^2 |
| Jerk | rad/s^3 |
| Torque | Nm |
| Current | A |
| Voltage | V |
| Temperature | degC |

Object dictionary values may be scaled integers. Convert at OD/application boundaries.

## Timing domains

| Domain | Rate | Main work |
|---|---:|---|
| Fast loop | 20 kHz | ADC current read, electrical angle read/predict, FOC update, PWM update |
| Medium loop | 1-2 kHz | SSI sample/state estimator, trajectory, position/velocity loops, current request |
| Slow loop | 10-100 Hz | OD sync, comms, faults, diagnostics, persistence service |

## Rate-crossing contract

- Medium loop writes torque/current command to a double-buffer or atomic copy/swap structure.
- Fast loop reads latest valid command without blocking.
- Slow loop may update requested objects, but mode manager applies them at controlled points.
- Fast loop never performs OD table search, SPI transactions, flash writes, or dynamic allocation.
- Persistent storage service only runs in slow/supervisory context.

## Generic motor torque request

The outer framework should use torque/current request structures rather than FOC-specific `iq` at public boundaries.

```c
typedef struct
{
    float torque_nm;
    float current_limit_a;
    bool enable;
} MC_MotorTorqueRequest_t;
```

For BLDC/FOC, convert torque to q-axis current using motor torque constant. For future brushed DC, convert torque to armature current.
