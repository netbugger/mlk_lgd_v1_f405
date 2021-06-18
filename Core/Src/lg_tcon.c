/*
 * lg_tcon.c
 *
 *  Created on: 2021. 6. 10.
 *      Author: netbugger
 */

#include "lg_tcon.h"
#include "printf.h"

tcon_frame_t TCON_FRAME[TCON_CH_NUM];
extern SPI_HandleTypeDef hspi2, hspi3;
extern DMA_HandleTypeDef hdma_spi2_rx, hdma_spi3_rx;


#if 1
void TCON_init(void)
{
	TCON_FRAME[0].pHspi = &hspi2;
	TCON_FRAME[0].pHdma = &hdma_spi2_rx;
	TCON_FRAME[0].complete = 0;
	TCON_FRAME[0].vsync = 1;

	TCON_FRAME[1].pHspi = &hspi3;
	TCON_FRAME[1].pHdma = &hdma_spi3_rx;
	TCON_FRAME[1].complete = 0;
	TCON_FRAME[1].vsync = 1;
}

#if 1
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	// CH 0
	tcon_frame_t *pFrame;
	if(VSYNCI_Pin & pin && !TCON_FRAME[0].vsync) {
//		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

		pFrame = &TCON_FRAME[0];
		//HAL_GPIO_TogglePin(GP_IO_GPIO_Port, GP_IO_Pin);
		//HAL_GPIO_TogglePin(GP_IO_GPIO_Port, GP_IO_Pin);
		HAL_SPI_Receive_DMA(pFrame->pHspi, pFrame->data, TCON_FRAME_LEN);
		pFrame->vsync = 1;
	}
	else if(VSYNCI2_Pin & pin && !TCON_FRAME[1].vsync) {
//		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		pFrame = &TCON_FRAME[1];
		HAL_SPI_Receive_DMA(pFrame->pHspi, pFrame->data, TCON_FRAME_LEN);
		pFrame->vsync = 1;
	}
	else {
		__asm volatile("NOP");
	}
}
#endif
#endif


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// CH 0
	tcon_frame_t *pFrame;
	if(hspi == TCON_FRAME[0].pHspi && !TCON_FRAME[0].complete) {
		pFrame = &TCON_FRAME[0];
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
		HAL_DMA_DeInit(pFrame->pHdma);
		HAL_SPI_DeInit(pFrame->pHspi);

		pFrame->complete = 1;
	}
	else if(hspi == TCON_FRAME[1].pHspi && !TCON_FRAME[1].complete){
		pFrame = &TCON_FRAME[1];
		HAL_GPIO_TogglePin(GP_IO_GPIO_Port, GP_IO_Pin);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		HAL_DMA_DeInit(pFrame->pHdma);
		HAL_SPI_DeInit(pFrame->pHspi);

		pFrame->complete = 1;
	}
	else {
		__asm volatile("NOP");
	}

}


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

#if 0
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
#endif
