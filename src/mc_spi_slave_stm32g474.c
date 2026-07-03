#include "mc_spi_slave.h"
#include "mc_comms.h"        /* MC_Comms_* */
#include "mc_if_protocol.h"  /* MC_IF_FRAME_SIZE (shared contract; add ../Generic_axis_controller/Generic_axis_controller/Interface to include path) */
#include "spi.h"             /* HAL: hspi2 -- boundary module */

/** @file mc_spi_slave_stm32g474.c
 *  @brief STM32G474 SPI2-slave DMA boundary (pipelined, double-buffered). See ADR-016 (F2b).
 *
 *  SPI2 = slave, 8-bit, mode 0, hardware NSS, RX/TX DMA. Each master transaction clocks exactly
 *  MC_IF_FRAME_SIZE (64) bytes.
 *
 *  Pipelined re-arm: the outbound frame is double-buffered. On transfer-complete we (1) swap so
 *  the already-prepared buffer becomes active, (2) **re-arm the DMA immediately** (just a buffer
 *  swap + arm, ~1-2 us), then (3) run the protocol handler to prepare the next outbound frame.
 *  This keeps the time-critical re-arm latency independent of the handler's work (OD access +
 *  telemetry gather), so the slave tolerates small / back-to-back inter-frame gaps (e.g. when the
 *  master bursts frames to catch up). Consequence: an OD response is pipelined by two transactions
 *  rather than one; the master correlates by sequence, so this is transparent.
 *
 *  On any SPI/DMA error or a failed re-arm it runs a robust reset (abort + force-reset both DMA
 *  handles + clear flags + state READY + re-arm), ported from bldc_axis_controller/spi2_slave.c.
 */

volatile MC_SpiSlaveStats_t g_spi_slave;

static uint8_t  s_rx[MC_IF_FRAME_SIZE]   __attribute__((aligned(4)));
static uint8_t  s_tx_a[MC_IF_FRAME_SIZE]  __attribute__((aligned(4)));
static uint8_t  s_tx_b[MC_IF_FRAME_SIZE]  __attribute__((aligned(4)));
static uint8_t *s_armed;     /* buffer currently being clocked out by the DMA */
static uint8_t *s_prepared;  /* buffer holding the next outbound frame */

static bool arm_dma(uint8_t *tx)
{
    /* Re-arm from a known-clean state: the HAL can leave a DMA handle (typically TX) marked BUSY
       when the master's NSS/SPE toggle races the TX-DMA completion, so HAL_DMA_Start_IT returns
       BUSY and the re-arm fails. Force both handles + the SPI to READY, idle SPE, clear OVR. */
    __HAL_SPI_DISABLE(&hspi2);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    if (hspi2.hdmatx != 0) { hspi2.hdmatx->State = HAL_DMA_STATE_READY; }
    if (hspi2.hdmarx != 0) { hspi2.hdmarx->State = HAL_DMA_STATE_READY; }
    hspi2.State = HAL_SPI_STATE_READY;

    const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&hspi2, tx, s_rx, MC_IF_FRAME_SIZE);
    g_spi_slave.last_rearm_hal = (uint32_t)st;
    return st == HAL_OK;
}

/* Robust recovery: clean the SPI + both DMA handles, then re-arm the active buffer. */
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

    if (!arm_dma(s_armed)) { g_spi_slave.rearm_fail++; }
}

void MC_SpiSlave_Init(void)
{
    MC_Comms_BuildIdle(s_tx_a);   /* both buffers hold a valid telemetry frame at startup */
    MC_Comms_BuildIdle(s_tx_b);
    s_armed    = s_tx_a;
    s_prepared = s_tx_b;
    if (!arm_dma(s_armed)) { reset_and_arm(); }
}

/* DMA transfer-complete: a full 64-byte transaction finished. SPI1 (SSI encoder) uses blocking
   transfers, so this callback is exclusive to SPI2. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        /* (1) the just-sent buffer is now free; (2) re-arm immediately with the prepared one. */
        uint8_t *just_sent = s_armed;
        s_armed = s_prepared;
        if (!arm_dma(s_armed))
        {
            g_spi_slave.rearm_fail++;
            reset_and_arm();
        }
        /* (3) process the received frame and prepare the freed buffer for the next-but-one tx. */
        MC_Comms_HandleTransaction(s_rx, just_sent);
        s_prepared = just_sent;
        g_spi_slave.transactions++;
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
        MC_Comms_BuildIdle(s_armed);   /* fresh outbound frame */
        reset_and_arm();
    }
}
