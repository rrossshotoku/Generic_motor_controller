# Open Assumptions and User-Supplied Details

The coding agent must not invent these final hardware details:

- ADC instances/channels for phase currents
- ADC trigger source and injected/regular configuration
- TIM1 vs TIM8 selection
- PWM channel polarity and complementary-output requirements
- dead-time value
- current-sense gains and offsets
- exact SSI SPI instance and frame timing
- exact SSI encoder frame layout for the first hardware
- inter-MCU SPI instance, master/slave role, and DMA strategy
- external network protocol used by network MCU
- exact motor electrical parameters and inertia values
- exact fault thresholds

The framework should expose configuration fields and wrapper APIs so these can be filled in later.
