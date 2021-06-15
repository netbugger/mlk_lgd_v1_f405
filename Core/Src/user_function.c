/*
 * user_function.c
 *
 *  Created on: Dec 21, 2020
 *      Author: soyul
 */

#include "stm32f4xx.h"

extern UART_HandleTypeDef huart5;
UART_HandleTypeDef *pUartPrint = &huart5;
void _putchar(char ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART3 and Loop until the end of transmission */
	if (ch == '\n') {
		HAL_UART_Transmit(pUartPrint, (uint8_t*) "\r", 1, 0xFFFF);
	}
	HAL_UART_Transmit(pUartPrint, (uint8_t*) &ch, 1, 0xFFFF);

	//return ch;
}


// Timer CallBack


