/*
 * config.c
 *
 *  Created on: 2021. 3. 19.
 *      Author: netbugger
 */
#include <string.h>
#include <test_mode.h>
#include <display.h>

config_t gConfig;
extern UART_HandleTypeDef huart4;
extern uint8_t dispBrightUp;
UART_HandleTypeDef *pUart = &huart4;

void load_config(void)
{
	memcpy(&gConfig, (config_t*)USER_DATA_BASE_ADDR, sizeof(config_t));
}

int save_config(void)
{
	FLASH_EraseInitTypeDef	erase;
	uint32_t	err;
	int i=0;
	uint64_t val;

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase.Sector = FLASH_SECTOR_11;
	erase.NbSectors = 1;

	HAL_FLASH_Unlock();
	if(HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
		return HAL_ERROR;
	}

	for(i=0; i < sizeof(config_t);  i+= sizeof(uint32_t)) {
		val = (uint64_t)(*((uint32_t*)(((void*)&gConfig + i))));
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA_BASE_ADDR+i, val) != HAL_OK) {
			return HAL_ERROR;
		}
	}

	HAL_FLASH_Lock();
	return 0;
}

int erase_config(void)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t err;

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase.Sector = FLASH_SECTOR_11;
	erase.NbSectors = 1;

	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
		return HAL_ERROR;
	}
	HAL_FLASH_Lock();

	return 0;
}

void TESTMODE_execute(void)
{
	int h, w;
	uint8_t val;
	uint8_t comm = 0;

	load_config();
	val = gConfig;
	while(!HAL_GPIO_ReadPin(TMODE_GPIO_Port, TMODE_Pin)) {
		for (h = 0; h < DISP_HEIGHT; h++) {
			for (w = 0; w < DISP_WIDTH; w++) {
				DISPLAY[h][w] = val * dispBrightUp;
			}
		}
		DISP_conv_to_FRAME();
		MLK_SPI_write_frame_data();

		// Waiting Remote Message
		HAL_UART_Receive(&huart4, (uint8_t *)&comm, 1, HAL_MAX_DELAY);
		switch(comm) {
		case HEADER_LOAD :
			//Read Dummy
			HAL_UART_Receive(&huart4, (uint8_t *)&val, 1, HAL_MAX_DELAY);
			val = gConfig;
			TESTMODE_send_data(HEADER_RESULT, val);
			break;

		case HEADER_SET :
			HAL_UART_Receive(&huart4, (uint8_t *)&val, 1, HAL_MAX_DELAY);
			gConfig = val;
			if(save_config() == 0) {
				HAL_GPIO_WritePin(GP_IO_GPIO_Port, GP_IO_Pin, GPIO_PIN_SET);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GP_IO_GPIO_Port, GP_IO_Pin, GPIO_PIN_RESET);
			}
			break;
		}
	}
}

void TESTMODE_send_data(uint8_t header, uint8_t payload)
{
	uint8_t data[2];

	data[0] = header;
	data[1] = payload;

	HAL_UART_Transmit(pUart, data, 2, 1000);
}
