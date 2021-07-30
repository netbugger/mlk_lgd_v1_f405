/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define UART_GND_Pin GPIO_PIN_3
#define UART_GND_GPIO_Port GPIOC
#define SDI4_Pin GPIO_PIN_2
#define SDI4_GPIO_Port GPIOA
#define SDI3_Pin GPIO_PIN_3
#define SDI3_GPIO_Port GPIOA
#define SDI2_Pin GPIO_PIN_7
#define SDI2_GPIO_Port GPIOA
#define X2_Pin GPIO_PIN_12
#define X2_GPIO_Port GPIOB
#define SCLK_Pin GPIO_PIN_13
#define SCLK_GPIO_Port GPIOB
#define X4_Pin GPIO_PIN_14
#define X4_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOC
#define X8_Pin GPIO_PIN_7
#define X8_GPIO_Port GPIOC
#define X16_Pin GPIO_PIN_8
#define X16_GPIO_Port GPIOC
#define TMODE_Pin GPIO_PIN_12
#define TMODE_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOC
#define IO_GND_Pin GPIO_PIN_11
#define IO_GND_GPIO_Port GPIOC
#define VSYNCI2_Pin GPIO_PIN_2
#define VSYNCI2_GPIO_Port GPIOD
#define VSYNCI2_EXTI_IRQn EXTI2_IRQn
#define SDI1_Pin GPIO_PIN_5
#define SDI1_GPIO_Port GPIOB
#define GP_IO_Pin GPIO_PIN_6
#define GP_IO_GPIO_Port GPIOB
#define VSYNC_Pin GPIO_PIN_8
#define VSYNC_GPIO_Port GPIOB
#define VSYNCI_Pin GPIO_PIN_9
#define VSYNCI_GPIO_Port GPIOB
#define VSYNCI_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
