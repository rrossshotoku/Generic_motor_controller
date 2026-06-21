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

## Realized (E1, ADR-018)

- **`mc_mode_manager.c`**: a simplified CiA-402 state machine driven by the contract's
  controlword bits (CW_ENABLE / CW_QUICK_STOP / CW_FAULT_RESET / CW_HALT) → Disabled / Ready /
  Enabled / Quick-Stop / Fault, producing the statusword + active `MC_Mode_t`. Severe fault latches
  until a fault-reset edge. Fault input is minimal (over-current trip) until the fault manager (E2).
- **Command arbitration** (scheduler medium loop): `inject_enable == true` → commissioning (the
  watch-window path drives, unchanged); `inject_enable == false` → remote (the mode manager,
  reading OD `0x6040/0x6060/0x60FF/0x6071`, drives). Boot-safe (controlword 0 → Disabled). One
  effective command feeds the fast/medium loops.
- **Routed modes**: Disabled, Profile/Joystick Velocity, Torque. Position Hold / Profile Position
  / Homing are recognised but not routed until the position loop + trajectory (D3) and calibration.
- **`mc_comms`** routes the cyclic command through the OD and reads the statusword from the OD —
  decoupled from the bring-up harness.
