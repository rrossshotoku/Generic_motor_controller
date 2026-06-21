#include "mc_spi_slave.h"
#include "mc_comms.h"        /* MC_Comms_* */
#include "mc_if_protocol.h"  /* MC_IF_FRAME_SIZE (shared contract; add ../Lightweight_CMC/Interface to include path) */
#include "spi.h"             /* HAL: hspi2 -- boundary module */

/** @file mc_spi_slave_stm32g474.c
 *  @brief STM32G474 SPI2-slave DMA boundary. See ADR-016 (F2b).
 *
 *  SPI2 = slave, 8-bit, mode 0, hardware NSS, RX/TX DMA. Each master transaction clocks exactly
 *  MC_IF_FRAME_SIZE (64) bytes; on completion the DMA cplt callback runs the protocol handler
 *  (which fills the next TX frame) and re-arms. On any SPI/DMA error or a failed re-arm it runs a
 *  robust reset (HAL_SPI_Abort + force-reset both DMA handles + clear flags + state READY +
 *  re-arm) -- without it a single overrun leaves the DMA BUSY/locked and every re-arm fails.
 *  Pattern ported from bldc_axis_controller/spi2_slave.c.
 */

volatile MC_SpiSlaveStats_t g_spi_slave;

static uint8_t s_rx[MC_IF_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx[MC_IF_FRAME_SIZE] __attribute__((aligned(4)));

static bool arm_dma(void)
{
    /* Re-arm from a known-clean state. By the time we re-arm, the previous transfer has
       completed (or been aborted), but the HAL can leave a DMA handle (typically TX) still
       marked BUSY -- the master's NSS/SPE toggle races the TX-DMA completion -- so
       HAL_DMA_Start_IT returns BUSY and the re-arm fails. Force both DMA handles + the SPI to
       READY, idle SPE (NSS-high), and clear any stale OVR before arming. */
    __HAL_SPI_DISABLE(&hspi2);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    if (hspi2.hdmatx != 0) { hspi2.hdmatx->State = HAL_DMA_STATE_READY; }
    if (hspi2.hdmarx != 0) { hspi2.hdmarx->State = HAL_DMA_STATE_READY; }
    hspi2.State = HAL_SPI_STATE_READY;
    return HAL_SPI_TransmitReceive_DMA(&hspi2, s_tx, s_rx, MC_IF_FRAME_SIZE) == HAL_OK;
}

/* Robust recovery: clean the SPI + both DMA handles, then re-arm. Safe to call from the
   SPI/DMA error ISR (transfer already stopped). */
static void reset_and_arm(void)
{
    g_spi_slave.resets++;

    HAL_SPI_Abort(&hspi2);

    if (hspi2.hdmatx != 0)
    {
        HAL_DMA_Abort(hspi2.hdmatx);
        hspi2.hdmatx->State     = HAL_DMA_STATE_READY;
        hspi2.hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
        __HAL_UNLOCK(hspi2.hdmatx);
    }
    if (hspi2.hdmarx != 0)
    {
        HAL_DMA_Abort(hspi2.hdmarx);
        hspi2.hdmarx->State     = HAL_DMA_STATE_READY;
        hspi2.hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
        __HAL_UNLOCK(hspi2.hdmarx);
    }

    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    __HAL_SPI_CLEAR_MODFFLAG(&hspi2);
    __HAL_SPI_CLEAR_FREFLAG(&hspi2);

    hspi2.State     = HAL_SPI_STATE_READY;
    hspi2.ErrorCode = HAL_SPI_ERROR_NONE;
    __HAL_UNLOCK(&hspi2);

    if (!arm_dma()) { g_spi_slave.rearm_fail++; }
}

void MC_SpiSlave_Init(void)
{
    MC_Comms_BuildIdle(s_tx);   /* valid telemetry frame ready for the first transaction */
    if (!arm_dma()) { reset_and_arm(); }
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
        if (!arm_dma())
        {
            g_spi_slave.rearm_fail++;
            reset_and_arm();
        }
    }
}

/* SPI error (overrun if the master mis-clocks, mode-fault on NSS glitch, etc.): robust reset. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        g_spi_slave.errors++;
        g_spi_slave.last_hal_error = hspi2.ErrorCode;
        if ((hspi2.ErrorCode & HAL_SPI_ERROR_OVR) != 0u) { g_spi_slave.err_overrun++; }
        MC_Comms_BuildIdle(s_tx);   /* fresh outbound frame */
        reset_and_arm();
    }
}
