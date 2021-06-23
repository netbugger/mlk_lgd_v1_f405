/*
 * ads1220.c
 *
 *  Created on: 2021. 2. 3.
 *      Author: netbugger
 */


#include <spi.h>
#include "main.h"
#include "dwt_stm32_delay.h"
#include "printf.h"

stm32_gpio_t SDI[MLK_PART_NUM];
mlk_frame_t FRAME;

//void __attribute__((optimize("O0")))MLK_SPI_init(void)
void MLK_SPI_init(void)
{
	uint8_t ic, part, scan, ch;
	SDI[0].port = SW_SPI_SDI1_PORT;
	SDI[0].pin = SW_SPI_SDI1_PIN;

	SDI[1].port = SW_SPI_SDI2_PORT;
	SDI[1].pin = SW_SPI_SDI2_PIN;

	SDI[2].port = SW_SPI_SDI3_PORT;
	SDI[2].pin = SW_SPI_SDI3_PIN;

	SDI[3].port = SW_SPI_SDI4_PORT;
	SDI[3].pin = SW_SPI_SDI4_PIN;

	// Init Frame Data
	for (ic = 0; ic < MLK_IC_NUM; ic++) {
		FRAME.ic[ic].dev = 0;
		FRAME.ic[ic].dev |= 0 << 15;	// Single Device
		FRAME.ic[ic].dev |= 0 << 14;	// Continuous Data
		FRAME.ic[ic].dev |= (ic + 1) << 8;	// Device Addr
		for (scan = 0; scan < MLK_SCAN_NUM; scan++) {
			FRAME.ic[ic].scan[scan].reg = MBI6334_REGADDR_SCAN1_CH56 + (scan * 0x40);
			for (part = 0; part < MLK_PART_NUM; part++) {
				for (ch = 0; ch < MLK_CHANNEL_NUM; ch++) {
					FRAME.ic[ic].scan[scan].part[part].ch[ch] = 0;
				}
			}
		}
	}
}


inline void __attribute__((optimize("O0")))MLK_SPI_set_pin(GPIO_TypeDef* port, uint16_t pin)
{
	port->BSRR = pin;
}

inline void __attribute__((optimize("O0")))MLK_SPI_reset_pin(GPIO_TypeDef* port, uint16_t pin)
{
#if HW_STMF405
	port->BSRR = pin << 16;
#else
	port->BRR = pin;
#endif
}

//inline void __attribute__((optimize("O0")))MLK_SPI_write_16bit_4ch(uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4)
inline void MLK_SPI_write_16bit_4ch(uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4)
{
	int i, j;
	//__disable_irq();
	for(i=0; i<16; i++) {
		if (val1 & (1 << (15 - i))) {
			MLK_SPI_set_pin(SDI[0].port, SDI[0].pin);
		}
		else {
			MLK_SPI_reset_pin(SDI[0].port, SDI[0].pin);
		}
		if (val2 & (1 << (15 - i))) {
			MLK_SPI_set_pin(SDI[1].port, SDI[1].pin);
		} else {
			MLK_SPI_reset_pin(SDI[1].port, SDI[1].pin);
		}
		if (val3 & (1 << (15 - i))) {
			MLK_SPI_set_pin(SDI[2].port, SDI[2].pin);
		} else {
			MLK_SPI_reset_pin(SDI[2].port, SDI[2].pin);
		}
		if (val4 & (1 << (15 - i))) {
			MLK_SPI_set_pin(SDI[3].port, SDI[3].pin);
		} else {
			MLK_SPI_reset_pin(SDI[3].port, SDI[3].pin);
		}
		MLK_SPI_set_pin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN);
		for(j=0; j<NOP_NUM; j++) {
			__asm volatile("NOP");
		}
		MLK_SPI_reset_pin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN);
		for(j=0; j<NOP_NUM; j++) {
			__asm volatile("NOP");
		}
	}
	HAL_GPIO_WritePin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN, GPIO_PIN_RESET);
	//__enable_irq();
}

inline void __attribute__((optimize("O1")))MLK_SPI_write_continous_data(continuous_data_t *pCont)
{
	int i;
	DWT_Delay_us(MBI6334_CS_DELAY_us);
	MLK_SPI_CS_LOW();

	//Dev Addr
	MLK_SPI_write_16bit_4ch(pCont->dev, pCont->dev, pCont->dev, pCont->dev);

	//NoD
	MLK_SPI_write_16bit_4ch(pCont->nData, pCont->nData, pCont->nData, pCont->nData);

	//RegAddr
	MLK_SPI_write_16bit_4ch(pCont->reg, pCont->reg, pCont->reg, pCont->reg);

	//Data
	for (i = 0; i < pCont->nData; i++) {
		MLK_SPI_write_16bit_4ch(pCont->pData[0][i], pCont->pData[1][i], pCont->pData[2][i], pCont->pData[3][i]);
	}

	//Dummy
	for (i = 0; i < MLK_IC_NUM - 1; i++) {
		MLK_SPI_write_16bit_4ch(0, 0, 0, 0);
	}
	MLK_SPI_CS_HIGH();

}

inline void __attribute__((optimize("O0")))MLK_SPI_write_single_data(uint16_t dev, uint16_t reg, uint16_t val)
{
	uint16_t data[3];
	int i,j,k;

	data[0] = dev;
	data[1] = reg;
	data[2] = val;

	DWT_Delay_us(MBI6334_CS_DELAY_us);
	MLK_SPI_CS_LOW();

	for (j = 0; j < 3; j++) {
		for (i = 0; i < 16; i++) {
			if (data[j] & (1 << (15 - i))) {
				MLK_SPI_set_pin(SDI[0].port, SDI[0].pin);
				MLK_SPI_set_pin(SDI[1].port, SDI[1].pin);
				MLK_SPI_set_pin(SDI[2].port, SDI[2].pin);
				MLK_SPI_set_pin(SDI[3].port, SDI[3].pin);
			} else {
				MLK_SPI_reset_pin(SDI[0].port, SDI[0].pin);
				MLK_SPI_reset_pin(SDI[1].port, SDI[1].pin);
				MLK_SPI_reset_pin(SDI[2].port, SDI[2].pin);
				MLK_SPI_reset_pin(SDI[3].port, SDI[3].pin);
			}
			MLK_SPI_set_pin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN);
			//for(k=0; k<NOP_NUM; k++) {
			//	__asm volatile("NOP");
			//}
			MLK_SPI_reset_pin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN);
			//for(k=0; k<NOP_NUM; k++) {
			//	__asm volatile("NOP");
			//}
		}
		HAL_GPIO_WritePin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN, GPIO_PIN_RESET);
	}

	// Send Dummy
	for (i = 0; i < MLK_IC_NUM -1; i++) {
		for (j = 0; j < 16; j++) {
#if HW_STMF405
				SDI[0].port->BSRR = SDI[0].pin<<16;
				SDI[1].port->BSRR = SDI[1].pin<<16;
				SDI[2].port->BSRR = SDI[2].pin<<16;
				SDI[3].port->BSRR = SDI[3].pin<<16;
#else
				SDI[0].port->BRR = SDI[0].pin;
				SDI[1].port->BRR = SDI[1].pin;
				SDI[2].port->BRR = SDI[2].pin;
				SDI[3].port->BRR = SDI[3].pin;
#endif
			SW_SPI_SCLK_PORT->BSRR = SW_SPI_SCLK_PIN;
#if HW_STMF405
			SW_SPI_SCLK_PORT->BSRR = SW_SPI_SCLK_PIN<<16;
#else
			SW_SPI_SCLK_PORT->BRR = SW_SPI_SCLK_PIN;
#endif
		}
	}
	HAL_GPIO_WritePin(SW_SPI_SCLK_PORT, SW_SPI_SCLK_PIN, GPIO_PIN_RESET);
	MLK_SPI_CS_HIGH();

}

//void __attribute__((optimize("O1")))MLK_SPI_write_frame_data(void)
void MLK_SPI_write_frame_data(void)
{
	uint8_t ic, scan, part;
	continuous_data_t cont;

	//DWT_Delay_us(MBI6334_CS_DELAY_us);

	//Write Dev
	for (ic = 0; ic < MLK_IC_NUM; ic++) {
		for (scan = 0; scan < MLK_SCAN_NUM; scan++) {
			cont.dev = FRAME.ic[ic].dev;
			cont.nData = MLK_CHANNEL_NUM;
			cont.reg =  FRAME.ic[ic].scan[scan].reg;
			for(part=0; part<4; part++) {
				cont.pData[part] = &(FRAME.ic[ic].scan[scan].part[part].ch[0]);
			}
			MLK_SPI_write_continous_data(&cont);
		}
	}

	MLK_SPI_write_single_data(MBI6334_BROADCAST|MBI6334_SINGLE_DATA|(MBI6334_DEVADDR_BROAD<<MBI6334_DEVADDR_OFFSET),
			MBI6334_FRAME_END_ADDR, MBI6334_FRAME_END_DATA);

	// VSYNC HIGH
		HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_SET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET);
}





//void __attribute__((optimize("O0")))MLK_DISP_config(void)
void MLK_DISP_config(void)
{
	int i;
	uint16_t cfg[16];
	continuous_data_t cont;

	cfg[0] = MLK_DEF_CONF1;
	cfg[1] = MLK_DEF_CONF2;
	cfg[2] = MLK_DEF_CONF3;
	cfg[3] = MLK_DEF_CONF4;
	cfg[4] = MLK_DEF_CONF5;
	cfg[5] = MLK_DEF_CONF6;
	cfg[6] = MLK_DEF_CONF7;
	cfg[7] = MLK_DEF_CONF8;
	cfg[8] = MLK_DEF_CONF9;
	cfg[9] = MLK_DEF_CONF10;
	cfg[10] = MLK_DEF_CONF11;
	cfg[11] = MLK_DEF_CONF12;
	cfg[12] = MLK_DEF_CONF13;
	cfg[13] = MLK_DEF_CONF14;
	cfg[14] = MLK_DEF_CONF15;
	cfg[15] = MLK_DEF_CONF16;


	//Send CF1
	cont.dev = MBI6334_BROADCAST | MBI6334_CONTINUOS | (MBI6334_DEVADDR_BROAD<<MBI6334_DEVADDR_OFFSET);
	cont.nData = 10;
	cont.reg = MBI6334_CONF1_ADDR;
	for(i=0; i<4; i++) {
		cont.pData[i] = cfg;
	}
	MLK_SPI_write_continous_data(&cont);

	//Send CF2
	cont.nData = 6;
	cont.reg = MBI6334_CONF1_ADDR + 10;
	for(i=0; i<4; i++) {
		cont.pData[i] = &(cfg[10]);
	}
	MLK_SPI_write_continous_data(&cont);

	MLK_SPI_write_frame_data();

}


#if 0
void mbi6334_write_reg(uint16_t reg, uint16_t val)
{
	int i;
	uint16_t devAddr = 0;
	uint16_t regAddr = 0;

	uint8_t data[16] = {0,};


	devAddr |= 0 << 15;	//To Single Device
	devAddr |= 1 << 14;	// Single Data
	devAddr |= 0x01 << 8;  // Device Address 0x01

	printf("[WRITE] devAddr = %04x\n", devAddr);

	regAddr |= 0x00 << 15; // Read Register
	regAddr |= reg;		// Register 3
	printf("[WRITE] regAddr = %04x\n", regAddr);


	data[0] = (uint8_t)(devAddr >> 8) & 0xff;
	data[1] = (uint8_t)(devAddr >> 0) & 0xff;
	data[2] = (uint8_t)(regAddr >> 8) & 0xff;
	data[3] = (uint8_t)(regAddr >> 0) & 0xff;
	data[4] = (uint8_t)(val >> 8) & 0xff;
	data[5] = (uint8_t)(val >> 0) & 0xff;

	LGD_CS_LOW();

	//while (HAL_SPI_GetState(pSpiHandle) != HAL_SPI_STATE_READY);
	//HAL_SPI_TransmitReceive(pSpiHandle, data, data+8, 8, 5);
	HAL_SPI_Transmit(pSpiHandle, data, 6, 5);
	//spi_write_16(devAddr);
	//spi_write_16(regAddr);
	//spi_write_16(0);
	//val[0] = spi_read_16();
	//val[1] = spi_read_16();
	//val[2] = spi_read_16();
	//val[0] = spi_read_16();
	//val[1] = spi_read_16();


	LGD_CS_HIGH();
#if 1
	for(i=0; i< 8; i++) {
		printf("%02x ", data[i]);
	}
	printf("\n");
	for(i=0; i< 8; i++) {
		printf("%02x ", data[8+i]);
	}
	printf("\n");
#endif
	//printf("val[0]=%x\n", val[0]);
	//printf("val[1]=%x\n", val[1]);
	//printf("val[2]=%x\n", val[2]);

}

void mbi6334_read_reg(uint16_t reg)
{
	int i;
	uint16_t devAddr = 0;
	uint16_t regAddr = 0;
	uint16_t val[3] ={0,};

	uint8_t data[16] = {0,};


	devAddr |= 0 << 15;	//To Single Device
	devAddr |= 1 << 14;	// Single Data
	devAddr |= 0x01 << 8;  // Device Address 0x01

	printf("[READ] devAddr = %04x\n", devAddr);

	regAddr |= 0x01 << 15; // Read Register
	regAddr |= reg;		// Register 3
	printf("[READ] regAddr = %04x\n", regAddr);


	data[0] = (uint8_t)(devAddr >> 8) & 0xff;
	data[1] = (uint8_t)(devAddr >> 0) & 0xff;
	data[2] = (uint8_t)(regAddr >> 8) & 0xff;
	data[3] = (uint8_t)(regAddr >> 0) & 0xff;
	LGD_CS_LOW();

	//while (HAL_SPI_GetState(pSpiHandle) != HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(pSpiHandle, data, data+8, 8, 5);
	//spi_write_16(devAddr);
	//spi_write_16(regAddr);
	//spi_write_16(0);
	//val[0] = spi_read_16();
	//val[1] = spi_read_16();
	//val[2] = spi_read_16();
	//val[0] = spi_read_16();
	//val[1] = spi_read_16();


	LGD_CS_HIGH();
#if 1
	for(i=0; i< 8; i++) {
		printf("%02x ", data[i]);
	}
	printf("\n");
	for(i=0; i< 8; i++) {
		printf("%02x ", data[8+i]);
	}
	printf("\n");
#endif
	//printf("val[0]=%x\n", val[0]);
	//printf("val[1]=%x\n", val[1]);
	//printf("val[2]=%x\n", val[2]);

}
#endif
