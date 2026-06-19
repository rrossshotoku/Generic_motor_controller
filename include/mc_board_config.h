#ifndef MC_BOARD_CONFIG_H
#define MC_BOARD_CONFIG_H
#include "mc_types.h"

/** @file mc_board_config.h
 *  @brief Per-board hardware configuration (power stage + sensing + wiring).
 *  @ingroup mc_core
 *
 *  All board-specific hardware values live here so the same framework binary can target
 *  different power boards by selecting a board profile. CubeMX still owns peripheral
 *  *initialisation*; the opaque handle fields below are wired by the STM32 boundary
 *  modules (mc_*_stm32g474.c) at startup. No HAL types appear in this header, so it stays
 *  host-compilable for tests. See ADR-004.
 */

/** @brief Position-feedback hardware backend selection. */
typedef enum
{
    MC_ENCODER_BACKEND_SSI = 0,        /**< Absolute SSI encoder over SPI. */
    MC_ENCODER_BACKEND_QUADRATURE = 1  /**< Incremental quadrature via timer (future). */
} MC_EncoderBackend_t;

/** @brief Phase-current shunt + amplifier + ADC scaling (three-shunt). */
typedef struct
{
    float    shunt_resistance_ohm;     /**< Shunt resistance [Ohm]. */
    float    amp_gain;                 /**< Current-sense amplifier gain [V/V]. */
    float    zero_current_offset_v;    /**< Amplifier output at zero current [V]. */
    float    adc_vref_v;               /**< ADC reference voltage [V]. */
    uint16_t adc_full_scale;           /**< ADC full-scale counts (e.g. 4096 for 12-bit). */
    int8_t   phase_sign[3];            /**< Per-phase measurement sign (+1/-1) from wiring/calibration. */
} MC_CurrentSenseConfig_t;

/** @brief DC-bus voltage sense divider scaling. */
typedef struct
{
    float    divider_ratio;            /**< Vbus = Vadc * divider_ratio [V/V]. BOARD VALUE - confirm. */
    float    adc_vref_v;               /**< ADC reference voltage [V]. */
    uint16_t adc_full_scale;           /**< ADC full-scale counts. */
} MC_BusSenseConfig_t;

/** @brief Power-stage / motor temperature sense scaling.
 *
 *  Profile 0 has no calibrated analog temperature sensor wired; temperature defaults to the
 *  model-based estimate (see motor thermal model). Populate these when a real sensor is used.
 *  BOARD VALUE - confirm. */
typedef struct
{
    bool  analog_sensor_present;       /**< true if a real analog temperature sensor is wired. */
    float scale_c_per_count;           /**< degC per ADC count. */
    float offset_c;                    /**< degC at zero counts. */
} MC_TempSenseConfig_t;

/** @brief 3-phase PWM power-stage timing. */
typedef struct
{
    uint32_t frequency_hz;             /**< Switching frequency [Hz]. */
    uint16_t period_counts;            /**< Timer ARR for centre-aligned PWM. */
    uint16_t deadtime_counts;          /**< Dead-time in timer counts. BOARD VALUE - confirm. */
    bool     complementary_outputs;    /**< true = high+low complementary drive. */
    bool     active_high;              /**< PWM output polarity. */
} MC_PwmConfig_t;

/** @brief Board-level position-feedback configuration.
 *
 *  The SSI frame bit-layout (data bits, error/warning/parity) lives in the SSI encoder
 *  module config; this captures only board-level facts. */
typedef struct
{
    MC_EncoderBackend_t backend;        /**< Which feedback backend this board uses. */
    uint32_t counts_per_rev;            /**< Mechanical counts per revolution. */
    uint8_t  resolution_bits;           /**< Encoder single-turn resolution in bits. */
    bool     invert_direction;          /**< true = invert so CW is positive. */
    float    mechanical_zero_offset_rad;/**< Mechanical zero offset [rad] (from calibration). */
} MC_BoardEncoderConfig_t;

/** @brief Complete per-board hardware profile. */
typedef struct
{
    const char *name;                   /**< Human-readable board profile name. */

    MC_CurrentSenseConfig_t current_sense;
    MC_BusSenseConfig_t     bus_sense;
    MC_TempSenseConfig_t    temp_sense;
    MC_PwmConfig_t          pwm;
    MC_BoardEncoderConfig_t encoder;

    /* Opaque CubeMX handle bindings, wired by the STM32 boundary modules at init.
       Cast to concrete HAL handle types inside those modules only. */
    void *htim_pwm;                     /**< Advanced timer for 3-phase PWM (TIM1). */
    void *hadc_phase_a;                 /**< ADC sampling phase A. */
    void *hadc_phase_c;                 /**< ADC sampling phase C. */
    void *hadc_vbus;                    /**< ADC sampling the DC bus (optional). */
    void *hspi_encoder;                 /**< SPI for the SSI encoder (SPI1). */
    void *hspi_link;                    /**< SPI for the inter-MCU link (SPI2, slave). */
} MC_BoardConfig_t;

/**
 * @brief Fill @p cfg with profile 0 (the proven reference board).
 *
 * Current-sense and PWM values are known-good. Fields marked "BOARD VALUE - confirm"
 * (bus divider, dead-time, temperature scaling) are placeholders to be set from the CubeMX
 * project / hardware before being relied upon. Handle pointers are left NULL for the
 * boundary modules to populate.
 * @param cfg Destination (must be non-NULL).
 */
void MC_BoardConfig_LoadProfile0(MC_BoardConfig_t *cfg);

/**
 * @brief Validate a board profile (positive scales, sane ADC full-scale, etc.).
 * @param cfg Profile to validate (must be non-NULL).
 * @return MC_OK if usable; an MC_ERR_* code otherwise.
 */
MC_Status_t MC_BoardConfig_Validate(const MC_BoardConfig_t *cfg);

/**
 * @brief Amps per ADC count for a current-sense configuration.
 *        amps_per_count = adc_vref_v / (adc_full_scale * amp_gain * shunt_resistance_ohm).
 * @param cs Current-sense configuration (must be non-NULL).
 * @return Amps per ADC count.
 */
float MC_CurrentSense_AmpsPerCount(const MC_CurrentSenseConfig_t *cs);

/**
 * @brief ADC count corresponding to zero current.
 *        zero_count = round(zero_current_offset_v / adc_vref_v * adc_full_scale).
 * @param cs Current-sense configuration (must be non-NULL).
 * @return ADC count at zero current.
 */
uint16_t MC_CurrentSense_ZeroCount(const MC_CurrentSenseConfig_t *cs);

#endif /* MC_BOARD_CONFIG_H */
