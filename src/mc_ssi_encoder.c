#include "mc_ssi_encoder.h"

/** @file mc_ssi_encoder.c
 *  @brief SSI absolute-encoder frame decode (board-independent). See ADR-008.
 *
 *  Extracts the position field from a raw SSI frame and produces a corrected single-turn
 *  mechanical angle (direction + zero offset applied). The hardware read lives in the STM32
 *  boundary (mc_ssi_encoder_stm32g474.c).
 */

#define MC_SSI_TWO_PI 6.28318530717958647692f

void MC_SsiEncoder_LoadDefaultConfig(MC_SsiEncoderConfig_t *cfg)
{
    /* AMM5B 21-bit single-turn over SPI1, read as 2 x 16-bit words -> 32-bit frame with the
       position at bits [30:10] (verified from bldc_axis_controller). */
    cfg->total_bits                 = 32u;
    cfg->position_bits              = 21u;
    cfg->position_lsb               = 10u;
    cfg->error_bit                  = -1;     /* none on AMM5B */
    cfg->warning_bit                = -1;
    cfg->parity_enabled             = false;
    cfg->parity_even                = false;
    cfg->counts_per_rev             = 2097152.0f;  /* 2^21 */
    cfg->mechanical_zero_offset_rad = 0.0f;
    cfg->direction                  = -1;     /* board: invert so CW is positive */
    cfg->sample_period_s            = 0.001f; /* read in the 1 kHz medium loop */
}

void MC_SsiEncoder_Init(MC_SsiEncoder_t *enc, const MC_SsiEncoderConfig_t *cfg)
{
    (void)cfg;
    enc->initialised              = false;
    enc->last_sample.position_rad = 0.0f;
    enc->last_sample.raw_position = 0u;
    enc->last_sample.timestamp_ticks = 0u;
    enc->last_sample.valid        = false;
    enc->last_sample.absolute     = true;
    enc->last_sample.error        = false;
    enc->last_sample.warning      = false;
}

bool MC_SsiEncoder_DecodeFrame(MC_SsiEncoder_t *enc, const MC_SsiEncoderConfig_t *cfg,
                               uint32_t raw_frame, MC_PositionSensorSample_t *sample)
{
    const uint32_t pos_mask = (cfg->position_bits >= 32u) ? 0xFFFFFFFFu
                                                          : ((1u << cfg->position_bits) - 1u);
    const uint32_t raw = (raw_frame >> cfg->position_lsb) & pos_mask;

    bool error = false;
    bool warning = false;
    if (cfg->error_bit >= 0)
    {
        error = ((raw_frame >> (uint8_t)cfg->error_bit) & 1u) != 0u;
    }
    if (cfg->warning_bit >= 0)
    {
        warning = ((raw_frame >> (uint8_t)cfg->warning_bit) & 1u) != 0u;
    }
    const bool valid = !error;   /* parity check omitted (AMM5B has none configured) */

    /* Single-turn mechanical angle [0, 2pi), with direction and zero offset applied. */
    float angle = (float)raw * (MC_SSI_TWO_PI / cfg->counts_per_rev);
    if (cfg->direction < 0)
    {
        angle = MC_SSI_TWO_PI - angle;
    }
    angle -= cfg->mechanical_zero_offset_rad;
    while (angle < 0.0f)            { angle += MC_SSI_TWO_PI; }
    while (angle >= MC_SSI_TWO_PI)  { angle -= MC_SSI_TWO_PI; }

    sample->raw_position = raw;
    sample->position_rad = angle;
    sample->valid        = valid;
    sample->absolute     = true;
    sample->error        = error;
    sample->warning      = warning;
    /* timestamp_ticks is set by the hardware read */

    enc->last_sample = *sample;
    return valid;
}
