# Basic Module Tests

The user will work with the coding agent interactively, so this is not a full verification plan. Still, each core module should have simple tests where practical.

## Suggested host-buildable tests

- PID: P output, PI integrator, clamp, reset, derivative filtering.
- OD: lookup, access denied, type mismatch, range reject, callbacks.
- SPI: CRC, encode/decode, bad sync, bad version, bad payload CRC.
- Trajectory: rest-to-rest, non-zero start velocity, time stretch reporting, replan failure preserves old plan.
- State estimator: wraparound handling, direction inversion, velocity filtering.
- FOC math: zero angle sanity, current PI saturation, SVPWM duty limits.
- Faults: debounce, latch, reset, severity action mapping.

## Hardware bring-up checks

- PWM outputs disabled by default.
- Current offsets calibrate with PWM disabled.
- Encoder position reads stable with motor stationary.
- Electrical angle changes correctly with mechanical rotation and pole-pair count.
- Fast loop can run at 20 kHz without blocking.
