# Architecture

## Functional blocks

- Network MCU object dictionary
- SPI inter-MCU transport
- Motor-control MCU object dictionary
- Mode manager / drive state machine
- Joystick conditioning
- Profile velocity conditioning
- Jerk-limited trajectory planner
- Position controller
- Velocity controller
- Torque/current request generator
- Motor-control backend
  - initial: BLDC/PMSM FOC backend
  - future: brushed DC backend
- Current measurement
- Position feedback backend
- State estimator
- Fault supervisor
- Limit manager
- Calibration manager
- Persistent parameter store
- Diagnostics

## Nested loop structure

```text
Position loop
    input: position demand, measured position
    output: velocity correction

Velocity loop
    input: velocity demand, measured velocity
    output: torque or iq-equivalent feedback correction

Current loop / FOC
    input: id/iq current command
    output: voltage vector / PWM duty cycles
```

## Command paths

### Profile position / timed move

```text
Trajectory Planner
    -> position demand
    -> velocity feedforward
    -> acceleration feedforward
    -> jerk diagnostics

Position Controller
    position error -> velocity correction

Velocity Demand Sum
    trajectory velocity feedforward + position correction

Velocity Controller
    velocity error -> torque/iq correction

Torque/Current Request Generator
    feedback correction + acceleration FF + friction + limits
    -> motor torque/current request
    -> BLDC FOC id/iq command
```

### Joystick velocity

```text
Joystick input
    -> deadband
    -> scaling
    -> acceleration limiting
    -> soft-limit slowdown
    -> Velocity Controller
```

Joystick mode bypasses trajectory planner and position controller, but not safety, conditioning, limits, or faults.

### Profile velocity

```text
OD target velocity
    -> velocity ramp / acceleration limit
    -> soft-limit slowdown
    -> Velocity Controller
```

### Torque/current mode

```text
OD torque/current command
    -> Torque/Current Request Generator
    -> Motor-control Backend
```

### Commissioning step injection

Protected commissioning mode can inject limited steps at:

- Position demand
- Velocity demand
- q-axis/current demand

This path is not available during normal remote operation and must be guarded by enable state, amplitude limits, timeout, and fault supervision.
