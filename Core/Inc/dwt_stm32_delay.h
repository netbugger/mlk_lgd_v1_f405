/*
 * dwt_stm32_delay.h
 *
 *  Created on: May 18, 2021
 *      Author: netbugger
 */

#ifndef DWT_STM32_DELAY_H_
#define DWT_STM32_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"

/**
 * @brief  Initializes DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: DWT counter Error
 *         0: DWT counter works
 */
uint32_t DWT_Delay_Init(void);



/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#if 0
__STATIC_INLINE void DWT_Delay_ns(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}
#endif


#ifdef __cplusplus
}
#endif

#endif /* DWT_STM32_DELAY_H_ */
