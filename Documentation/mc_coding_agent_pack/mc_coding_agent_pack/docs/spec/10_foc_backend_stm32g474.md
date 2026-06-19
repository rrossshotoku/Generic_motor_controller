# BLDC/PMSM FOC Backend

## Initial backend

- Three-shunt BLDC/PMSM FOC
- STM32G474RET3
- PWM-triggered ADC current sampling
- TIM1 or TIM8 centre-aligned 3-phase PWM
- SSI absolute encoder used for electrical angle

## FOC processing

Inputs:

- `id_command_a`
- `iq_command_a`
- phase currents `ia`, `ib`, `ic`
- electrical angle
- DC bus voltage
- enable/inhibit flags

Processing:

1. Clarke transform phase currents to alpha/beta.
2. Park transform alpha/beta to d/q using electrical angle.
3. d-axis PI current control.
4. q-axis PI current control.
5. Voltage vector limiting.
6. Anti-windup on saturated voltage vector.
7. Inverse Park transform voltage d/q to alpha/beta.
8. SVPWM duty calculation.
9. Output duty cycles to PWM backend.

## Excluded from first implementation

- Field weakening
- MTPA
- Overmodulation
- Sensorless observer
- Single-shunt reconstruction
- Advanced current reconstruction edge cases

## Current measurement

Current-sense backend shall provide currents in amps and validity flags. ADC channels, injected/regular choice, and trigger details are CubeMX/user configuration, but the framework should assume PWM-synchronised sampling.

## PWM backend

PWM backend shall provide:

```c
void MC_Pwm_Start(void);
void MC_Pwm_Stop(void);
void MC_Pwm_SetDutyFast(const MC_PwmDuty_t *duty);
void MC_Pwm_ForceSafeOff(void);
```

Dead-time is configured by CubeMX. Break input/emergency shutdown should be supported if configured.
