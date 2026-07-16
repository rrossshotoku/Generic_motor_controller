/*
 * boot_stubs — symbols the Cube-generated code references but the bootloader
 * must define itself.
 *
 *   SystemClock_Config — verbatim copy of Core/Src/main.c's version so the
 *                        bootloader boots the same 170 MHz PLL as the app.
 *   Error_Handler      — halt on unrecoverable HAL error.
 *   SysTick_Handler    — HAL time base (drives HAL_Delay / HAL_GetTick).
 *
 * Core/Src/stm32g4xx_it.c is deliberately NOT compiled into the bootloader —
 * it references app-only peripheral handles (hadc*, htim*, hdma_*). We supply
 * the minimal interrupt set here.
 */

#include "stm32g4xx_hal.h"

void Error_Handler(void);   /* defined below; forward-declared for the callers above it */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* halt for debugger */ }
}

/* --- Minimal interrupt handlers ---------------------------------------- */
void SysTick_Handler(void) { HAL_IncTick(); }

void NMI_Handler        (void) { while (1) {} }
void HardFault_Handler  (void) { while (1) {} }
void MemManage_Handler  (void) { while (1) {} }
void BusFault_Handler   (void) { while (1) {} }
void UsageFault_Handler (void) { while (1) {} }
void SVC_Handler        (void) { /* no RTOS */ }
void DebugMon_Handler   (void) { /* no debug monitor */ }
void PendSV_Handler     (void) { /* no RTOS */ }

/* --- SPI2-slave DMA interrupt plumbing (boot_spi.c owns the handles) ------
 * These drive HAL_SPI_TxRxCpltCallback / HAL_SPI_ErrorCallback (boot_main.c),
 * which run the double-buffered re-arm. */
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

void DMA1_Channel5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi2_rx); }
void DMA1_Channel6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi2_tx); }
void SPI2_IRQHandler(void)          { HAL_SPI_IRQHandler(&hspi2); }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    Error_Handler();
}
#endif
