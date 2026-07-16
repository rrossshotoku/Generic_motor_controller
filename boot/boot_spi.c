/*
 * boot_spi — SPI2 SLAVE + DMA init for the bootloader.
 *
 * Mirrors Core/Src/spi.c's MX_SPI2_Init + the SPI2 DMA MspInit exactly
 * (SPI_MODE_SLAVE, 8-bit, CPOL low / CPHA 1-edge, hardware NSS input; RX =
 * DMA1_Channel5 (DMA_REQUEST_SPI2_RX), TX = DMA1_Channel6 (DMA_REQUEST_SPI2_TX)).
 * The bootloader drives the slave with HAL_SPI_TransmitReceive_DMA + immediate
 * re-arm in the transfer-complete callback (boot_main.c), so a valid mc_if frame
 * is staged on the MISO for every 1 kHz master transaction.
 *
 * hspi2 / hdma_spi2_* share the app's global names; declared here because the
 * app's spi.c is not compiled into the bootloader.
 */

#include "stm32g4xx_hal.h"

void Error_Handler(void);   /* boot_stubs.c */

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_SLAVE;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 7;
    hspi2.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) { return; }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI, AF5. */
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* DMA clocks. */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* SPI2_RX -> DMA1_Channel5. */
    hdma_spi2_rx.Instance                 = DMA1_Channel5;
    hdma_spi2_rx.Init.Request             = DMA_REQUEST_SPI2_RX;
    hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode                = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx);

    /* SPI2_TX -> DMA1_Channel6. */
    hdma_spi2_tx.Instance                 = DMA1_Channel6;
    hdma_spi2_tx.Init.Request             = DMA_REQUEST_SPI2_TX;
    hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode                = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hspi, hdmatx, hdma_spi2_tx);

    /* DMA + SPI2 NVIC (priority 3 — the bootloader has no higher-rate ISRs). */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_SetPriority(SPI2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
}
