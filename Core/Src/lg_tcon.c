/*
 * lg_tcon.c
 *
 *  Created on: 2021. 6. 10.
 *      Author: netbugger
 */

#include "lg_tcon.h"
#include "printf.h"

tcon_frame_t TCON_FRAME;
#if 0
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	//static int vsync = 0;
	// VSYNC FALLING
	static uint8_t data = 0;
	static int idx = 0;

	if(TCON_VSYNCI_Pin & pin) {
		//vsync = 1;
		// CLOCK Interrupt Start
		//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		idx = 0;
		TCON_FRAME.idx = 0;
	}
	else if( TCON_SCKI_Pin & pin) {
#if 0
		if ((TCON_DATAI_GPIO_Port->IDR & TCON_DATAI_Pin) != (uint32_t)GPIO_PIN_RESET) {
		    data |= (1<<(7-idx));
		}
		idx++;
		if(idx == 8) {
			//DATA Handling
			TCON_add_data(data);
			data = 0;
			idx = 0;
		}
#endif
		HAL_GPIO_TogglePin(GP_IO_GPIO_Port, GP_IO_Pin);
	}
}
#endif

void TCON_add_data(uint8_t val)
{
	uint8_t *pData = (uint8_t *)TCON_FRAME.data;
	uint8_t chksum;
	int i;
	if(TCON_FRAME.idx != 0) {
		pData[TCON_FRAME.idx-1] = val;
		TCON_FRAME.idx++;
		if(TCON_FRAME.idx == TCON_CHECKSUM_IDX) {
			chksum = TCON_FRAME.indicator ^ TCON_FRAME.checksum;
			for(i = 0; i < TCON_DATA_LEN; i++) {
				chksum ^= pData[i];
			}
			if(chksum == val) {
				//FRAME COPY
				printf("ind=%x, chksum=%x\n", TCON_FRAME.indicator, chksum);
			}
			else {
				//Log
				printf("[F]ind=%x, chksum=%x\n", TCON_FRAME.indicator, chksum);
			}
			TCON_FRAME.idx=0;
		}
	}
	else {
		//Check if Indicator
		TCON_FRAME.indicator = val;
		TCON_FRAME.idx++;
	}
}
