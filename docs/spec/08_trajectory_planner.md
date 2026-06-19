# Trajectory Planner

## Algorithm

First implementation shall be a jerk-limited S-curve planner.

Requirements:

- velocity limit
- acceleration limit
- deceleration limit
- jerk limit
- requested target time
- time stretching if requested time is infeasible
- reporting of requested time vs planned time
- replan while active
- non-zero start velocity support
- zero target velocity and zero target acceleration initially

## Boundary-condition scope

Initial implementation supports:

- arbitrary start position
- non-zero start velocity
- normally zero start acceleration, or planned acceleration if available
- target position
- target velocity = 0
- target acceleration = 0

The interface shall contain target velocity and target acceleration fields, but the first implementation may reject non-zero target velocity/acceleration with `MC_TRAJ_ERR_UNSUPPORTED_BOUNDARY`.

## Timing behaviour

The planner shall attempt to finish at `requested_time_s`.

If not physically feasible:

- stretch the time to the minimum feasible duration or a suitable longer duration
- set `time_stretched = true`
- return `MC_TRAJ_TIME_STRETCHED`
- expose `planned_time_s`

## Output

Each update returns:

- position demand
- velocity feedforward
- acceleration feedforward
- jerk
- complete flag
- valid flag

## Replan behaviour

If a new trajectory is requested while active:

1. Evaluate current planned trajectory state at current elapsed time.
2. Use that state as the new start state by default.
3. Attempt to generate the new plan.
4. If successful, atomically replace the active plan.
5. If unsuccessful, keep the old trajectory active and report error.

## Implementation note

The first S-curve implementation may use a conservative robust approach. It is more important that it is bounded, stable, and correctly reports stretching than that it handles every mathematically optimal case.
