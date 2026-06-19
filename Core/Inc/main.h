/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define I_C_Pin GPIO_PIN_0
#define I_C_GPIO_Port GPIOC
#define I_B_Pin GPIO_PIN_1
#define I_B_GPIO_Port GPIOC
#define ADC_DRIVE_SIG_Pin GPIO_PIN_2
#define ADC_DRIVE_SIG_GPIO_Port GPIOC
#define I_A_Pin GPIO_PIN_0
#define I_A_GPIO_Port GPIOA
#define ADC_VBUS_Pin GPIO_PIN_1
#define ADC_VBUS_GPIO_Port GPIOA
#define nPWM_PHA_Pin GPIO_PIN_7
#define nPWM_PHA_GPIO_Port GPIOA
#define UART_DE_Pin GPIO_PIN_4
#define UART_DE_GPIO_Port GPIOC
#define nPWM_PHB_Pin GPIO_PIN_0
#define nPWM_PHB_GPIO_Port GPIOB
#define nPWM_PHC_Pin GPIO_PIN_1
#define nPWM_PHC_GPIO_Port GPIOB
#define GPO_1_Pin GPIO_PIN_7
#define GPO_1_GPIO_Port GPIOC
#define WIZ_RSTn_Pin GPIO_PIN_8
#define WIZ_RSTn_GPIO_Port GPIOC
#define EXTI19_WIZ_Pin GPIO_PIN_9
#define EXTI19_WIZ_GPIO_Port GPIOC
#define PWM_PHA_Pin GPIO_PIN_8
#define PWM_PHA_GPIO_Port GPIOA
#define PWM_PHB_Pin GPIO_PIN_9
#define PWM_PHB_GPIO_Port GPIOA
#define PWM_PHC_Pin GPIO_PIN_10
#define PWM_PHC_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_15
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOB
#define GPO_2_Pin GPIO_PIN_6
#define GPO_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
