# Mode Manager and Drive State Machine

## Supported modes

- Disabled
- Position Hold
- Profile Position / Timed Move
- Profile Velocity (live demand from the cyclic `velocity_setpoint`, v3)
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
| Profile Velocity | live `velocity_setpoint` (cyclic, v3) -> velocity loop -> current loop |
| Torque/Current | torque/current request -> motor backend |
| Homing/Calibration | calibration manager owns command path, safety-gated |
| Quick Stop | high-priority controlled decel if possible |
| Fault | action depends on severity |

## Stop behaviour

- Normal stop / `velocity_setpoint` to zero: controlled decel to zero velocity, then position hold.
- Quick stop: higher-priority controlled decel if feedback/control valid.
- Severe fault: immediate PWM/current disable where necessary.

## Mode-change rules

- Changing into profile position while moving shall use trajectory replan from current planned/measured state as appropriate.
- Changing from a velocity mode to hold shall controlled-decelerate to zero, capture stopped position, then hold.
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
- **Routed modes**: Disabled, Profile Velocity, Torque. Position Hold / Profile Position / Homing
  are recognised but not routed until the position loop + trajectory (D3) and calibration.
- **`mc_comms`** routes the cyclic command through the OD and reads the statusword from the OD —
  decoupled from the bring-up harness.

## Update (v3 cyclic command, ADR-021)

- **JOYSTICK velocity mode removed** (REQ-0010). Joystick is a CMC `axis_manager` concept, not a
  motor primitive; the CMC's JOYSTICK op-mode maps to motor `PROFILE_VELOCITY` + a streamed
  velocity. `MC_MODE_JOYSTICK_VELOCITY` is gone from the enum, the mode-manager `switch`, and the
  scheduler routing.
- **Velocity demand source**: in `PROFILE_VELOCITY` the live demand is the cyclic `velocity_setpoint`
  (lands in `0x60FF`; the velocity loop consumes it). Mode + other targets are SDO-owned and persist.
- **NEW_SETPOINT**: the scheduler decodes `MC_IF_CW_NEW_SETPOINT` into `dc.new_setpoint`; the mode
  manager flags its rising edge as `ds.new_setpoint_latched` (one-shot). The D3 trajectory engine
  will consume it to start a PROFILE_POSITION move (`0x607B` timing); execution is deferred to D3.
