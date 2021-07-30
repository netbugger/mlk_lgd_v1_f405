/*
 * config.c
 *
 *  Created on: 2021. 3. 19.
 *      Author: netbugger
 */
#include "config.h"
#include <string.h>

config_t gConfig;

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
