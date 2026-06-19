# STM32G474RET3 Integration

## CubeMX ownership

The user owns CubeMX setup:

- Clock tree
- GPIO configuration
- ADC channel assignment
- TIM1/TIM8 PWM configuration
- SPI configuration for SSI and inter-MCU link
- DMA configuration
- NVIC priorities
- HAL initialisation

The coding agent may integrate using `USER CODE` sections and framework wrapper files, but must not overwrite CubeMX-generated setup.

## HAL/LL policy

- HAL is acceptable for setup, slow operations, and non-critical wrappers.
- LL or direct registers may be used in timing-critical ADC/PWM fast-loop wrappers.
- Keep LL/register code isolated in STM32 boundary modules, for example:
  - `mc_current_sense_stm32g474.c`
  - `mc_pwm_stm32g474.c`
  - `mc_ssi_encoder_stm32g474.c`
  - `mc_fast_loop_stm32g474.c`
  - `mc_cordic_stm32g474.c`

## CORDIC/FM​​​​AC

The STM32G474 has mathematical accelerators such as CORDIC/FM​​​​AC. Use them only behind wrapper functions. The algorithm should have a pure C fallback.

Suggested wrapper:

```c
void MC_Math_SinCos(float angle_rad, float *s, float *c);
float MC_Math_Atan2(float y, float x);
float MC_Math_Sqrt(float x);
```

The FOC module should call these wrappers rather than depending directly on HAL CORDIC APIs.
