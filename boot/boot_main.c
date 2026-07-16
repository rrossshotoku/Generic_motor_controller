/*
 * Bootloader entry point for the motor MCU.
 *
 * Boots at 0x08000000. On the persistent boot flag (0x08008000): CLEAR + a valid
 * app -> jump to the app at 0x08008800; STAY (or a bad app image) -> serve a
 * firmware update over the inter-MCU SPI2 slave link.
 *
 * SPI: DMA double-buffer, exactly like the app's mc_spi_slave_stm32g474.c — a
 * valid mc_if frame is staged on the MISO for EVERY 1 kHz master transaction
 * (the CMC counts any unframed MISO as an error). On transfer-complete we swap +
 * re-arm immediately, then run boot_od_on_frame() to prepare the next frame. The
 * slow PROG_START erase runs in the main loop (boot_od_pump), not the ISR.
 *
 * Design ref: Interface/REQUESTS.md REQ-0015 + the CMC-side bring-up feedback.
 */

#include "boot_flag.h"
#include "boot_od.h"

#include "mc_if_protocol.h"      /* MC_IF_FRAME_SIZE */
#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_SPI2_Init(void);
extern SPI_HandleTypeDef hspi2;

#define APP_FLASH_BASE     0x08008800u
#define APP_STACK_ADDR     (*(volatile uint32_t *)(APP_FLASH_BASE + 0u))
#define APP_RESET_VECTOR   (*(volatile uint32_t *)(APP_FLASH_BASE + 4u))

void boot_jump_to_app(void);

/* Double-buffered TX + single RX (mirror mc_spi_slave). */
static uint8_t  s_rx[MC_IF_FRAME_SIZE]     __attribute__((aligned(4)));
static uint8_t  s_rx_snap[MC_IF_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t  s_tx_a[MC_IF_FRAME_SIZE]  __attribute__((aligned(4)));
static uint8_t  s_tx_b[MC_IF_FRAME_SIZE]  __attribute__((aligned(4)));
static uint8_t *s_armed;      /* buffer being clocked out now      */
static uint8_t *s_prepared;   /* buffer holding the next frame     */

static bool arm_dma(uint8_t *tx)
{
    __HAL_SPI_DISABLE(&hspi2);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    if (hspi2.hdmatx != 0) { hspi2.hdmatx->State = HAL_DMA_STATE_READY; }
    if (hspi2.hdmarx != 0) { hspi2.hdmarx->State = HAL_DMA_STATE_READY; }
    hspi2.State = HAL_SPI_STATE_READY;
    return HAL_SPI_TransmitReceive_DMA(&hspi2, tx, s_rx, MC_IF_FRAME_SIZE) == HAL_OK;
}

static void reset_and_arm(void)
{
    HAL_SPI_Abort(&hspi2);
    if (hspi2.hdmatx != 0) {
        HAL_DMA_Abort(hspi2.hdmatx);
        hspi2.hdmatx->State = HAL_DMA_STATE_READY; hspi2.hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
        __HAL_UNLOCK(hspi2.hdmatx);
    }
    if (hspi2.hdmarx != 0) {
        HAL_DMA_Abort(hspi2.hdmarx);
        hspi2.hdmarx->State = HAL_DMA_STATE_READY; hspi2.hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
        __HAL_UNLOCK(hspi2.hdmarx);
    }
    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    __HAL_SPI_CLEAR_MODFFLAG(&hspi2);
    __HAL_SPI_CLEAR_FREFLAG(&hspi2);
    hspi2.State = HAL_SPI_STATE_READY; hspi2.ErrorCode = HAL_SPI_ERROR_NONE;
    __HAL_UNLOCK(&hspi2);
    (void)arm_dma(s_armed);
}

/* A full 64-byte transaction finished: swap + re-arm immediately, then prepare
 * the freed buffer for the next-but-one transaction. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) { return; }
    /* Snapshot the received frame BEFORE re-arming. The handler can take up to
     * ~0.6 ms (per-segment flash program); once the DMA is re-armed it refills
     * s_rx with the next frame, so processing s_rx directly would let it be
     * overwritten mid-flight (corrupts seg_length/flags/data). */
    memcpy(s_rx_snap, s_rx, MC_IF_FRAME_SIZE);
    uint8_t *just_sent = s_armed;
    s_armed = s_prepared;
    if (!arm_dma(s_armed)) { reset_and_arm(); }
    boot_od_on_frame(s_rx_snap, just_sent);
    s_prepared = just_sent;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) { return; }
    boot_od_build_idle(s_armed);
    reset_and_arm();
}

static bool app_image_looks_valid(void)
{
    uint32_t sp = APP_STACK_ADDR;
    return (sp >= 0x20000000u) && (sp <= 0x20020000u);
}

/* Jump to the app WITHOUT resetting (see the brick-proof rationale in
 * mc_boot_meta.h). Re-enables PRIMASK before branching so app SysTick works. */
void boot_jump_to_app(void)
{
    __disable_irq();
    HAL_DeInit();
    for (int i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SCB->VTOR = APP_FLASH_BASE;
    __DSB(); __ISB();

    uint32_t app_sp    = APP_STACK_ADDR;
    uint32_t app_entry = APP_RESET_VECTOR;
    __enable_irq();
    __set_MSP(app_sp);
    ((void (*)(void))app_entry)();
    for (;;) { __NOP(); }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Keep the hot ISR path (SPI DMA callback + boot_od) running from the
     * instruction cache while flash is being erased/programmed, so the SPI
     * slave keeps answering the master through a download. */
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    MX_GPIO_Init();

    if (!boot_flag_is_stay()) {
        if (app_image_looks_valid()) {
            boot_jump_to_app();
        }
        /* Bad app image — fall through to bootloader mode to recover. */
    }

    MX_SPI2_Init();
    boot_od_init();

    boot_od_build_idle(s_tx_a);
    boot_od_build_idle(s_tx_b);
    s_armed = s_tx_a; s_prepared = s_tx_b;
    if (!arm_dma(s_armed)) { reset_and_arm(); }

    /* Heartbeat LED on PB11: 500 ms on / 500 ms off (1 s period) -- TWICE the app's blink
     * rate (the app uses a 2 s period), so "in bootloader" is visually distinct. Configured
     * here (serve path only): if we jumped to the app above, the app owns PB11 instead. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    {
        GPIO_InitTypeDef led = {0};
        led.Pin   = GPIO_PIN_11;
        led.Mode  = GPIO_MODE_OUTPUT_PP;
        led.Pull  = GPIO_NOPULL;
        led.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &led);
    }

    for (;;) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,
                          ((HAL_GetTick() / 500u) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        boot_od_pump();                 /* deferred app-region erase (INIT) */
        if (boot_od_commit_pending()) {
            HAL_Delay(100);             /* let the COMMIT OK drain out via DMA */
            boot_jump_to_app();         /* no return */
        }
    }
}
