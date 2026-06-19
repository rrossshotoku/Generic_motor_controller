# Tests

Start with host-buildable tests for pure C modules:

- `mc_pid`
- `mc_spi_protocol`
- `mc_od`
- `mc_trajectory` planning logic
- `mc_math`

Hardware boundary modules can use fake ADC/PWM/SSI providers during host tests.
