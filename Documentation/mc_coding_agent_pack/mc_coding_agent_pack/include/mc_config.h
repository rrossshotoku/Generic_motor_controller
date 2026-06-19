#ifndef MC_CONFIG_H
#define MC_CONFIG_H

/** @file mc_config.h
 *  @brief Build-time configuration defaults for the motor-controller framework.
 */

#define MC_FAST_LOOP_HZ        (20000u)
#define MC_MOTION_LOOP_HZ      (1000u)
#define MC_SLOW_LOOP_HZ        (100u)

#define MC_FAST_DT_S           (1.0f / (float)MC_FAST_LOOP_HZ)
#define MC_MOTION_DT_S         (1.0f / (float)MC_MOTION_LOOP_HZ)
#define MC_SLOW_DT_S           (1.0f / (float)MC_SLOW_LOOP_HZ)

#define MC_SPI_PROTOCOL_VERSION (1u)
#define MC_SPI_MAX_PAYLOAD      (256u)

#define MC_ENABLE_COMMISSIONING_STEP_TESTS (1u)
#define MC_ENABLE_CORDIC_WRAPPER           (1u)

#endif
