# Real-Time Scheduling

## Fast loop: 20 kHz

Called from PWM/ADC timing domain after current samples are ready.

Responsibilities:

1. Read phase currents from current-sense fast buffer.
2. Read or predict electrical angle.
3. Read latest double-buffered current command.
4. Apply fast fault inhibit.
5. Run FOC current controller.
6. Update PWM duty cycles.
7. Force safe-off on severe fault.

Prohibited:

- Blocking HAL calls
- SPI object access
- Flash writes
- printf/log formatting
- Dynamic allocation
- Long table searches

## Medium loop: 1-2 kHz

Responsibilities:

1. Read position sensor sample.
2. Update state estimator.
3. Update command conditioner or trajectory planner.
4. Run position loop when active.
5. Run velocity loop when active.
6. Run torque/current request generator.
7. Publish command to fast loop buffer.
8. Check medium-rate faults such as following error and encoder validity.

## Slow loop: 10-100 Hz

Responsibilities:

1. Object dictionary service.
2. SPI link state supervision.
3. Mode-manager slow actions.
4. Thermal/bus-voltage derating.
5. Diagnostics snapshots.
6. Persistent store save/load service.
7. Calibration state machines that do not require fast timing.
