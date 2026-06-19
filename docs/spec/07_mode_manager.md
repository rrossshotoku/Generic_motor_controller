# Mode Manager and Drive State Machine

## Supported modes

- Disabled
- Position Hold
- Profile Position / Timed Move
- Joystick Velocity
- Profile Velocity
- Torque/Current Mode
- Homing/Calibration
- Quick Stop
- Fault

## Mode routing

| Mode | Command path |
|---|---|
| Disabled | PWM/current disabled |
| Position Hold | Captured hold position -> position loop |
| Profile Position / Timed Move | trajectory -> position loop -> velocity loop -> current loop |
| Joystick Velocity | joystick conditioning -> velocity loop -> current loop |
| Profile Velocity | velocity conditioner -> velocity loop -> current loop |
| Torque/Current | torque/current request -> motor backend |
| Homing/Calibration | calibration manager owns command path, safety-gated |
| Quick Stop | high-priority controlled decel if possible |
| Fault | action depends on severity |

## Stop behaviour

- Normal stop / joystick release: controlled decel to zero velocity, then position hold.
- Quick stop: higher-priority controlled decel if feedback/control valid.
- Severe fault: immediate PWM/current disable where necessary.

## Mode-change rules

- Changing into profile position while moving shall use trajectory replan from current planned/measured state as appropriate.
- Changing from joystick/profile velocity to hold shall controlled-decelerate to zero, capture stopped position, then hold.
- Torque/current mode shall be commissioning/safety gated if it bypasses outer loops.
- Fault state latches until reset conditions are met.
