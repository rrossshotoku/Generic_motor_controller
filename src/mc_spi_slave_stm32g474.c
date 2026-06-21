#include "mc_spi_slave.h"
#include "mc_comms.h"        /* MC_Comms_* */
#include "mc_if_protocol.h"  /* MC_IF_FRAME_SIZE (shared contract; add ../Lightweight_CMC/Interface to include path) */
#include "spi.h"             /* HAL: hspi2 -- boundary module */

/** @file mc_spi_slave_stm32g474.c
 *  @brief STM32G474 SPI2-slave DMA boundary. See ADR-016 (F2b).
 *
 *  SPI2 is configured (CubeMX) as slave, 8-bit, mode 0, hardware NSS, with RX/TX DMA. Each
 *  master transaction clocks exactly MC_IF_FRAME_SIZE (64) bytes; on completion the DMA cplt
 *  callback runs the protocol handler (which fills the next TX frame) and re-arms. The handler
 *  runs at the SPI DMA IRQ priority (below the control loops), does only bounded non-blocking
 *  work, and must complete + re-arm within the master's inter-frame gap (~1 ms at 1 kHz).
 */

volatile MC_SpiSlaveStats_t g_spi_slave;

static uint8_t s_rx[MC_IF_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx[MC_IF_FRAME_SIZE] __attribute__((aligned(4)));

static void arm(void)
{
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, s_tx, s_rx, MC_IF_FRAME_SIZE) != HAL_OK)
    {
        g_spi_slave.rearm_fail++;
    }
}

void MC_SpiSlave_Init(void)
{
    MC_Comms_BuildIdle(s_tx);   /* have a valid telemetry frame ready for the first transaction */
    arm();
}

/* DMA transfer-complete: a full 64-byte transaction finished. SPI1 (SSI encoder) uses blocking
   transfers, so this callback is exclusive to SPI2. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        /* Process the received frame and build the frame to send on the next transaction. */
        MC_Comms_HandleTransaction(s_rx, s_tx);
        g_spi_slave.transactions++;
        arm();
    }
}

/* SPI error (e.g. overrun if the master mis-clocks): recover and re-arm. First-cut recovery;
   refine on-target if errors are seen. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        g_spi_slave.errors++;
        (void)HAL_SPI_Abort(&hspi2);
        MC_Comms_BuildIdle(s_tx);
        arm();
    }
}
