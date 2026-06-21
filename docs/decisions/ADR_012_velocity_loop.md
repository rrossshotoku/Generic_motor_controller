# ADR-012: Velocity loop + torque/current request (D2)

## Status

Accepted

## Date

2026-06-21

## Context

Under fixed iq (D1) an unloaded motor runs to the bus-voltage speed limit. The velocity loop
lets us command rad/s and hold a speed. The proven loop output current directly
(Kp 150 A/(rad/s), Ki 1000, ±nominal current). The generic architecture instead has the
velocity loop output **torque (Nm)**, and a **current-request generator** convert torque→iq via
Kt — so outer layers stay motor-type-agnostic (a future brushed-DC backend maps torque→armature
current instead).

## Decision

- **`mc_velocity_controller.c`**: PI (reusable MC_Pid) on velocity error → torque correction,
  derivative-on-measurement, integrator+output clamp = torque limit.
- **`mc_current_request.c`**: torque = velocity correction + inertia·accel_ff + friction_ff,
  clamp to torque limit → `MC_MotorTorqueRequest_t`; then iq = torque/Kt clamped to the current
  limit, id = 0.
- **Gains**: ported from the proven loop, expressed in torque = old·Kt (Kp 150·0.231 ≈ 34.65,
  Ki 1000·0.231 = 231; output limit = current·Kt). Net iq is identical to the proven loop.
- **Cadence/rate-crossing**: the velocity cascade runs in the **1 kHz medium loop** (after the
  estimator) and publishes the iq command to the **20 kHz fast loop** via a single
  `volatile float` (atomic on M4). The fast-loop FOC uses the published iq in velocity mode,
  else the manual iq (D1). Feedback = the estimator velocity (observer by default, ADR-003).
- **Modes**: `velocity_enable` selects the velocity loop; gated by `inject_enable && foc_enable`
  and the over-current trip. Accel/friction feedforward are present but **0** until the
  trajectory (D3) and friction identification land.
- **Limits**: velocity-loop current limit 2.5 A; over-current trip default raised to 3.0 A for
  breakaway headroom (housing breakaway ≈ 1.5–2 A).

## Reasoning

Expressing the proven tuning in torque preserves the validated behaviour while honouring the
motor-agnostic seam. Running the loop at 1 kHz and publishing one atomic float matches the old
architecture (velocity at 1 kHz feeding FOC at 20 kHz) and avoids a full double-buffer.

## Consequences

- New `mc_velocity_controller.c`, `mc_current_request.c`; scheduler gains a velocity cascade in
  the medium loop + an iq publish to the fast loop; new watch/inject fields (velocity_enable,
  velocity_cmd_rad_s; vel_demand, vel_torque_cmd_nm, vel_iq_cmd_a).
- Velocity controller resets on mode entry (no windup while idle).
- The torque model (inertia, Kt, friction, limits) comes from the default motor model.

## Files affected

- src/mc_velocity_controller.c, src/mc_current_request.c
- include/mc_debug.h, src/mc_scheduler.c
- docs/spec/09_control_loops.md

## Open questions

- Final velocity gains for this load (tune on bench).
- Friction/accel feedforward identification (with D3 trajectory).
- Velocity-loop output slew limit, if needed.
