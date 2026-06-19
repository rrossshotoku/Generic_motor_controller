#include "mc_ssi_encoder.h"
#include "mc_debug.h"   /* MC_Debug_Cycles() for the sample timestamp */
#include "spi.h"        /* HAL: hspi1 (SPI1 master, 16-bit, Mode 2) -- boundary module */

/** @file mc_ssi_encoder_stm32g474.c
 *  @brief STM32G474 SSI read over SPI1 (2 x 16-bit words -> 32-bit frame). See ADR-008.
 *
 *  SPI1 is configured (CubeMX) as master, 16-bit, CPOL=1/CPHA=0, MSB-first, ~1.33 MHz. The
 *  TX bytes are don't-care (we only need the clock burst); the encoder shifts the position
 *  out on MISO. Decoding is delegated to the board-independent MC_SsiEncoder_DecodeFrame.
 */

bool MC_SsiEncoder_ReadHardware(MC_SsiEncoder_t *enc, const MC_SsiEncoderConfig_t *cfg,
                                MC_PositionSensorSample_t *sample)
{
    uint16_t tx[2] = { 0u, 0u };
    uint16_t rx[2] = { 0u, 0u };

    /* Blocking 32-bit clock burst (~24 us). Called from the 1 kHz medium loop. */
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, (uint8_t *)rx, 2u, 2u) != HAL_OK)
    {
        sample->valid = false;
        sample->error = true;
        return false;
    }

    const uint32_t frame = ((uint32_t)rx[0] << 16) | (uint32_t)rx[1];
    const bool ok = MC_SsiEncoder_DecodeFrame(enc, cfg, frame, sample);
    sample->timestamp_ticks = MC_Debug_Cycles();
    enc->last_sample = *sample;
    return ok;
}
