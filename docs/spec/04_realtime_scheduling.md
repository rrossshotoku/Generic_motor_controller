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

## Realized scheduling (implementation, see ADR-006)

Concrete mapping on this board (STM32G474, CubeMX timers):

- **Fast 20 kHz** — driven from the **ADC end-of-conversion** ISR. The ADC is
  hardware-triggered by **TIM1 TRGO = OC4REF** (CH4 = 4249, the counter peak of the
  centre-aligned ARR-4250 PWM), so the sample lands in the low-side conduction window and the
  fast loop runs once per PWM period, synchronised to the current sample. TIM1 runs as the
  PWM time base to generate the trigger; its update interrupt is not used.
- **Medium 1 kHz** — driven from the **TIM7 update** event (PSC 169, ARR 999).
- **Slow 100 Hz** — decimated from the medium loop (÷10) and serviced in the **main loop**
  via `MC_Sched_ServiceBackground`, keeping longer work out of ISR context.

Dispatch (in `main.c` USER CODE): `HAL_ADC_ConvCpltCallback` (ADC1) → `MC_Sched_FastTick`,
and `HAL_TIM_PeriodElapsedCallback` (TIM7) → `MC_Sched_MediumTick`. The `mc_scheduler` module
stays HAL-free; `main.c` (the HAL boundary) starts TIM1 (time base, for the trigger), TIM7,
and the ADC. PWM pin outputs remain disabled (safe-off) until deliberately enabled. Loop
cadence and worst-case durations are mirrored in `g_mc_debug` for the watch window.
