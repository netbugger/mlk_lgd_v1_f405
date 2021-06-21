/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "printf.h"
#include "dwt_stm32_delay.h"
#include "spi.h"
#include "lg_tcon.h"
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t gVsyncFlag = 0;
extern tcon_frame_t TCON_FRAME[];
extern uint16_t DISPLAY[DISP_HEIGHT][DISP_WIDTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void TCON_SPI_RESET(void);
void TCON_SPI_RESET2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i = 0, j = 0, k = 0, l = 0;
	uint8_t chksum[2];
	uint16_t linebuffer[48];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  TCON_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	DWT_Delay_Init();
	MLK_SPI_init();

	printf("HI\n");
	// Wait 10ms
	HAL_Delay(10);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	MLK_DISP_config();
	HAL_TIM_Base_Start_IT(&htim6);
	TCON_FRAME[0].vsync = 0;
	TCON_FRAME[1].vsync = 0;
//	DISP_conv_to_FRAME();
//	MLK_SPI_write_frame_data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//printf("Hello\n");
		//HAL_Delay(1000);
#if 1
		if(TCON_FRAME[0].complete && TCON_FRAME[1].complete) {
			uint8_t c = 0;
			if(TCON_FRAME[0].data[TCON_OFFSET_INDICATOR] != TCON_INDICATOR_VAL || TCON_FRAME[1].data[TCON_OFFSET_INDICATOR] != TCON_INDICATOR_VAL) {
				printf("indicator fail\n");
				TCON_SPI_RESET();
			}
			else {
				//printf("SPI : [%x, %x], [%x, %x]\n", TCON_FRAME[0].data[TCON_OFFSET_INDICATOR], TCON_FRAME[0].data[TCON_OFFSET_CHKSUM],
				//		TCON_FRAME[1].data[TCON_OFFSET_INDICATOR], TCON_FRAME[1].data[TCON_OFFSET_CHKSUM]);

				//DWT_Delay_us(500);
				// Check Sum
				chksum[0] = TCON_FRAME[0].data[TCON_OFFSET_INDICATOR] ^ TCON_FRAME[0].data[TCON_OFFSET_CMD];
				chksum[1] = TCON_FRAME[1].data[TCON_OFFSET_INDICATOR] ^ TCON_FRAME[1].data[TCON_OFFSET_CMD];
				for(i=2; i<TCON_FRAME_LEN-1; i++) {
					chksum[0] ^= TCON_FRAME[0].data[i];
					chksum[1] ^= TCON_FRAME[1].data[i];

				}

				// Verify checksum
				if(chksum[0] != TCON_FRAME[0].data[TCON_OFFSET_CHKSUM] || chksum[1] != TCON_FRAME[1].data[TCON_OFFSET_CHKSUM] ) {
					printf("Checksum fail\n");
					TCON_SPI_RESET();
				}
				else {
					// Transfer Data
					TCON_conv_to_DISPLAY();
					DISP_conv_to_FRAME();


					MLK_SPI_write_frame_data();
					TCON_SPI_RESET2();

		//			TCON_FRAME[0].complete = 0;
		//			TCON_FRAME[1].complete = 0;
		//			TCON_FRAME[0].vsync = 0;
		//			TCON_FRAME[1].vsync = 0;
					// Send To MBI
				}
			}
		}
		else {
			__asm volatile("NOP");
			i = TCON_FRAME[0].complete;
			j = TCON_FRAME[1].complete;
			k = TCON_FRAME[0].vsync;
			l = TCON_FRAME[1].vsync;
		}
#endif

#if 0
		//if (gVsyncFlag != 0) {
			gVsyncFlag = 0;
			DISP_conv_to_FRAME();
			MLK_SPI_write_frame_data();
			j++;
			if(j == 5) {
				j = 0;
				if(i) {
					memcpy(DISPLAY[i-1], linebuffer, sizeof(uint16_t)*48);
				}
				memcpy(linebuffer, DISPLAY[i], sizeof(uint16_t)*48);
				memset(DISPLAY[i], 0x00, sizeof(uint16_t)*48);
				i++;
				if(i == 42) {
					i = 0;
				}
			}
		//}

		//else {
		//	__asm volatile("NOP");
		//}
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (840-1);
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1666;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SDI4_Pin|SDI3_Pin|SDI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCLK_Pin|SDI1_Pin|GP_IO_Pin|GPIO_PIN_7
                          |VSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|EN_Pin|UART_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SDI4_Pin SDI3_Pin SDI2_Pin */
  GPIO_InitStruct.Pin = SDI4_Pin|SDI3_Pin|SDI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SCLK_Pin SDI1_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|SDI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin UART_GND_Pin */
  GPIO_InitStruct.Pin = EN_Pin|UART_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNCI2_Pin */
  GPIO_InitStruct.Pin = VSYNCI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VSYNCI2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GP_IO_Pin PB7 VSYNC_Pin */
  GPIO_InitStruct.Pin = GP_IO_Pin|GPIO_PIN_7|VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNCI_Pin */
  GPIO_InitStruct.Pin = VSYNCI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VSYNCI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
inline void TCON_SPI_RESET(void)
{
	uint8_t c = 0;
//	HAL_DMA_DeInit(TCON_FRAME[0].pHdma);
//	HAL_DMA_DeInit(TCON_FRAME[1].pHdma);
//	HAL_SPI_DeInit(TCON_FRAME[0].pHspi);
//	HAL_SPI_DeInit(TCON_FRAME[1].pHspi);

	MX_DMA_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();

	HAL_SPI_TransmitReceive(TCON_FRAME[0].pHspi, &c, &c, 1, 1000);
	HAL_SPI_TransmitReceive(TCON_FRAME[1].pHspi, &c, &c, 1, 1000);

	TCON_FRAME[0].complete = 0;
	TCON_FRAME[0].vsync = 0;
	TCON_FRAME[1].complete = 0;
	TCON_FRAME[1].vsync = 0;

	 HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
		  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

		  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
		  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

inline void TCON_SPI_RESET2(void)
{
//	HAL_DMA_DeInit(TCON_FRAME[0].pHdma);
//	HAL_DMA_DeInit(TCON_FRAME[1].pHdma);
//	HAL_SPI_DeInit(TCON_FRAME[0].pHspi);
//	HAL_SPI_DeInit(TCON_FRAME[1].pHspi);

	MX_DMA_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();

//	HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
//	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
	TCON_FRAME[0].complete = 0;
	TCON_FRAME[0].vsync = 0;
	TCON_FRAME[1].complete = 0;
	TCON_FRAME[1].vsync = 0;
	  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}
void VSYNC_set_freq(int hz)
{
	uint32_t prescaler = (640-1);
	uint32_t timerClock = 64000000;
	uint32_t counter;

	counter =  timerClock/((prescaler+1)*hz) -1;
	__HAL_TIM_SET_COUNTER(&htim6, (uint16_t)counter);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		gVsyncFlag = 1;
	}
	else {
		__asm volatile("NOP");
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
