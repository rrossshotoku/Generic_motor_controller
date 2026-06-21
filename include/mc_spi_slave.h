#ifndef MC_SPI_SLAVE_H
#define MC_SPI_SLAVE_H
#include <stdint.h>

/** @file mc_spi_slave.h
 *  @brief STM32G474 SPI2-slave DMA transport for the inter-MCU link (F2b). See ADR-016.
 *
 *  Drives the fixed 64-byte full-duplex frames over SPI2 (slave, hardware NSS) using DMA, and
 *  hands each completed transaction to the HAL-free protocol handler (mc_comms). On any SPI/DMA
 *  error or a failed re-arm it runs a robust reset (abort + force-ready + clear flags + re-arm),
 *  ported from the proven bldc_axis_controller SPI2 slave.
 */

/** @brief Transport observability (watch window). */
typedef struct
{
    uint32_t transactions;   /**< Completed 64-byte transactions. */
    uint32_t errors;         /**< SPI error-callback events (overrun, mode-fault, etc.). */
    uint32_t rearm_fail;     /**< DMA re-arm failures. */
    uint32_t resets;         /**< Robust resets performed. */
    uint32_t err_overrun;    /**< Of `errors`, those that were overruns (OVR). */
    uint32_t last_hal_error; /**< Last hspi2.ErrorCode (HAL_SPI_ERROR_*). */
} MC_SpiSlaveStats_t;

extern volatile MC_SpiSlaveStats_t g_spi_slave;

/** @brief Arm the SPI2-slave DMA. Call after MX_SPI2_Init + MC_Framework_Init (MC_Comms_Init). */
void MC_SpiSlave_Init(void);

#endif /* MC_SPI_SLAVE_H */
