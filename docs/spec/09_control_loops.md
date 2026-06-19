# Control Loops

## Reusable PID/PI primitive

Implement a reusable PID block used by:

- Position controller
- Velocity controller
- FOC d-axis current PI
- FOC q-axis current PI

Features:

- P/PI/PID through config flags
- derivative filtering
- derivative on measurement preferred where appropriate
- integrator clamp
- output clamp
- reset/enable behaviour
- optional slew limiting at wrapper level

## Position controller

Architecture:

```text
position error = position_demand - position_actual
velocity_correction = PID(position error)
velocity_demand = trajectory_velocity_ff + velocity_correction
```

Default: P-only, but configurable PID.

## Velocity controller

Architecture:

```text
velocity error = velocity_demand - velocity_actual
torque_or_iq_correction = PID/PI(velocity error)
```

Default: PI, but configurable PID.

## Torque/current request generator

Inputs:

- velocity-loop feedback correction
- trajectory acceleration feedforward
- measured velocity
- motor/axis model parameters
- active current/torque limits
- derating state

Processing:

```text
torque_ff = inertia * acceleration_ff
friction_ff = static_friction * sign(velocity) + viscous_friction * velocity
torque_request = velocity_feedback_correction + torque_ff + friction_ff
apply torque/current/thermal limits
```

For BLDC/FOC:

```text
iq_command = torque_request / Kt
id_command = 0 initially
```

## Current controller / FOC PI loops

FOC shall use two PI loops:

- d-axis current PI
- q-axis current PI

Outputs are voltage commands `vd`, `vq`. The vector shall be limited against available bus voltage. Apply current-loop anti-windup when voltage saturation occurs.
