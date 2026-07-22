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

**Soft max-demand current limit (ADR-069):** the demanded current at the current-loop input shall be
clampable to a working ceiling *below* the hard over-current trip (`current_trip_a`, `0x2600:2`), so
the axis saturates at a chosen current instead of tripping. `current_demand_limit_a` (`0x2400:8`, RW
PERSIST, `0` = disabled) clamps the torque-producing command (iq for FOC, armature for brushed) to
`±limit` in the fast loop, downstream of every command source (velocity/position cascade, torque
mode, and the tuning sweep). It bounds `iq` (≈ peak phase current when `id ≈ 0`, matching the OC-trip
quantity); `id` is left unclamped, still backstopped by the trip. Distinct from the velocity loop's
`vel_current_limit_a` (`0x2300:4`), which only limits velocity/position-derived commands.

## Realized (implementation)

- **Current loop / FOC** (D1, ADR-011): `mc_foc.c` — Clarke/Park, d/q PI (kp 1.7, ki 1700,
  ±24 V), circular voltage limit, inverse Park, SVPWM. Runs at 20 kHz.
- **Velocity loop** (D2, ADR-012): `mc_velocity_controller.c` — PI on velocity error → torque
  correction; gains ported from the proven loop expressed in torque (= old·Kt: kp ≈ 34.65,
  ki = 231; output limit = current·Kt). Runs at 1 kHz; feedback = estimator velocity (observer
  default). Output iq published to the fast loop as an atomic float. The applied gains are
  `vel_kp·load` / `vel_ki·load` where `load = clamp(vel_load_factor, 0.3, 2.0)` (`0x2300:5`, default
  1.0) — an operator load multiplier the CMC's payload slider drives (ADR-034).
- **Torque/current request** (D2, ADR-012): `mc_current_request.c` — torque = velocity
  correction + inertia·accel_ff + friction_ff, clamped; iq = torque/Kt clamped to the current
  limit, id = 0. Accel/friction FF present but 0 until D3 / friction ID.
- **Position loop** (D3, ADR-028): `mc_position_controller.c` — `velocity_correction =
  PID(position_demand − position_actual)`, P-only default (gains `0x2200`), output clamped to a
  velocity-correction limit, error clamped to a following-error limit. A **position-error deadband**
  `position_deadband_rad` (`0x2200:5`, ADR-071, 0 = off) nulls the correction within ±deadband of the
  target so the axis parks instead of hunting; continuous form (subtracts the band outside it → the
  correction reaches 0 smoothly at the edge, no velocity step), applied before the following-error
  clamp, on the loop's error only (raw `position_error_rad` telemetry is unaffected). Keep it below
  the target-reached window (0.01 rad). Wired into the scheduler
  `PROFILE_POSITION` cascade: `NEW_SETPOINT` starts a trapezoidal plan (target `0x607A`, time
  `0x607B`, limits `0x6081/3/4`); each 1 kHz tick trajectory → position loop →
  `velocity_demand = velocity_ff_gain·trajectory_velocity_ff + correction` → velocity loop (the FF gain
  `0x2200:4`, default 1.0, trims the feedforward ratio; ADR-031); `accel_ff` →
  the torque request's inertia slot. `target_reached` (complete + within window) → statusword
  bit `MC_IF_SW_TARGET_REACHED (0x0400)`. First cut: from-rest, single fixed window.

## Loop tuning — on-motor test-signal modes (ADR-030)

For tuning, a motor-owned **`test_mode`** overlay (OD `0x2910:1`: 0 off / 1 velocity-tuning /
2 position-tuning) injects a clean, motor-generated reference at the loop under test, while the matching
operational mode is **enabled** (the overlay reuses the normal enable + over-current trip + CMC drive
state; it does not self-enable):

- **Velocity-tuning** (`PROFILE_VELOCITY` enabled): the generator output replaces the velocity-loop demand
  `s_eff_vel_cmd` (absolute, from 0).
- **Position-tuning** (`PROFILE_POSITION` enabled): the generator output, added to the position captured on
  entry, replaces the position demand into the position controller, **bypassing the trajectory** (vel-FF 0)
  — so the position loop is tested with a raw reference rather than a planned move.

The reference comes from a shared pure module **`mc_signal_gen`** (`IDLE → RAMP → DWELL → RAMP→0`; `rate = 0`
⇒ step edge; `continuous` ⇒ alternating ping-pong train). Units follow the domain (velocity: amplitude
rad/s, rate rad/s²; position: amplitude rad, rate rad/s). Triggered via `0x2910:6`; clearing `test_mode`
ramps bumplessly to 0. Graph `tlm_vel_demand_rad_s` (0x2310:1) vs `tlm_vel_actual_rad_s` (0x2310:2) to read
the response. Tune the inner velocity loop first, then the outer position loop.
