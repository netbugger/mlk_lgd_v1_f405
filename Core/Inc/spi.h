/*
 * ads1220.h
 *
 *  Created on: 2021. 2. 3.
 *      Author: netbugger
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include <stdint.h>
#include "main.h"

#define HW_STMF405			1
#define HW_SPI				0

#define MLK_SPI_DEF_TIMEOUT		5

#define MLK_IC_NUM				3
#define MLK_CHANNEL_NUM			56
#define MLK_SCAN_NUM			3
#define MLK_PART_NUM			4


#define MBI6334_CS_DELAY_us		2
#define MBI6334_VSYNC_PERIOD_us	1

#define MBI6334_REGADDR_MASK0	0x400
#define MBI6334_REGADDR_MASK3	0x403

#define MBI6334_BROADCAST		0x8000
#define MBI6334_DEVTARGET		0x0000
#define MBI6334_SINGLE_DATA		0x4000
#define MBI6334_CONTINUOS		0x0000

#define MBI6334_DEVADDR_OFFSET	8
#define MBI6334_DEVADDR_BROAD	0

// Configuration 1
// 0000 0000 0100 0000			SCAN 3line, pwm continous
//#define LGD_DEF_CONF1	0x0040
//#define LGD_DEF_CONF1	0x0000
//Config 1
#define MBI6334_CONF1_OV_3_5V	0x0000
#define MBI6334_CONF1_OV_4_0V	0x4000
#define MBI6334_CONF1_OV_4_5V	0x8000
#define MBI6334_CONF1_OV_5_0V	0xC000

#define MBI6334_CONF1_SCRAMBLE_1	0x0000
#define MBI6334_CONF1_SCRAMBLE_8	0x0100
#define MBI6334_CONF1_SCRAMBLE_16	0x0200
#define MBI6334_CONF1_SCRAMBLE_32	0x0300

#define MBI6334_CONF1_SCAN_1		0x0000
#define MBI6334_CONF1_SCAN_2		0x0020
#define MBI6334_CONF1_SCAN_3		0x0040
#define MBI6334_CONF1_SCAN_4		0x0060
#define MBI6334_CONF1_SCAN_5		0x0080
#define MBI6334_CONF1_SCAN_6		0x00A0
#define MBI6334_CONF1_SCAN_7		0x00C0
#define MBI6334_CONF1_SCAN_8		0x00E0

#define MBI6334_CONF1_PWM_CONTINU		0x0000
#define MBI6334_CONF1_PWM_ONESHOT		0x0010

#define MBI6334_CONF1_MODE_PWM		0x0000
#define MBI6334_CONF1_MODE_PAM		0x000F

#define MLK_DEF_CONF1	(MBI6334_CONF1_OV_5_0V | MBI6334_CONF1_SCRAMBLE_1 | MBI6334_CONF1_SCAN_3 | MBI6334_CONF1_PWM_CONTINU)
//#define MLK_DEF_CONF1	(MBI6334_CONF1_OV_5_0V | MBI6334_CONF1_SCRAMBLE_1 | MBI6334_CONF1_SCAN_3 | MBI6334_CONF1_PWM_ONESHOT)
#define MBI6334_CONF1_ADDR	0x0000
//#define MLK_DEF_CONF1	0xC150


//Configuration 2
#define MLK_DEF_CONF2	0x0800	//FDC ON

//Configuration 3
//#define MLK_DEF_CONF3	0x0019
#define MLK_DEF_CONF3	0x0018

//Configuration 4
//#define MLK_DEF_CONF4	0xC8C8
#define MLK_DEF_CONF4	0x0808		//DM DT

//Configuration 5
//#define MLK_DEF_CONF5	0x0000
#define MLK_DEF_CONF5	0x3307	//GCG(Global current gain) : 0x33(51), 5+(GCG/255)*25 mA = 6mA

//Configuration 6
#define MBI6334_CONF6_DEGHOST_1		0x0000
#define MBI6334_CONF6_DEGHOST_2		0x2000
#define MBI6334_CONF6_DEGHOST_3		0x4000
#define MBI6334_CONF6_DEGHOST_4		0x6000
#define MBI6334_CONF6_DEGHOST_5		0x8000
#define MBI6334_CONF6_DEGHOST_6		0xA000
#define MBI6334_CONF6_DEGHOST_7		0xC000
#define MBI6334_CONF6_DEGHOST_8		0xE000
//#define MLK_DEF_CONF6	0x0000
#define MLK_DEF_CONF6	0xF600

//Configuration 7
#define MLK_DEF_CONF7	0x0000

//Configuration 8
#define MLK_DEF_CONF8	0x0000	//Scan Line ST, CT

//Configuration 9
#define MBI6334_CONF9_FBO_DISABLE		0x0000
#define MBI6334_CONF9_FBO_ENABLE		0x8000
//Open dectection Function
#define MBI6334_CONF9_ODF_DISABLE		0x0000
#define MBI6334_CONF9_ODF_ENALBE		0x4000
//Short Dectection Function
#define MBI6334_CONF9_SDF_DISABLE		0x0000
#define MBI6334_CONF9_SDF_ENALBE		0x2000
//Error Detect Update Number
#define MBI6334_CONF9_EDUN_1			0x0000
#define MBI6334_CONF9_EDUN_2			0x0400
#define MBI6334_CONF9_EDUN_3			0x0800
#define MBI6334_CONF9_EDUN_4			0x0C00
#define MBI6334_CONF9_EDUN_5			0x1000
#define MBI6334_CONF9_EDUN_6			0x1400
#define MBI6334_CONF9_EDUN_7			0x1800
#define MBI6334_CONF9_EDUN_8			0x1C00
//FBO update Frequency
#define MBI6334_CONF9_FBOUF_1			0x0000		//every frame
#define MBI6334_CONF9_FBOUF_2			0x0100		//every 2frame
#define MBI6334_CONF9_FBOUF_3			0x0200		//every 3frame
#define MBI6334_CONF9_FBOUF_4			0x0300		//every 4frame
// LED Error Mask
#define MBI6334_CONF9_LEDEM_DISABLE		0x0000
#define MBI6334_CONF9_LEDEM_ENABLE		0x0040
//Interrupt Enable
#define MBI6334_CONF9_INT_DISABLE		0x0000
#define MBI6334_CONF9_INT_ENABLE		0x0020
//Open Dectection Voltage Level
#define MBI6334_CONF9_ODVL_0_10			0x0000
#define MBI6334_CONF9_ODVL_0_15			0x0004
#define MBI6334_CONF9_ODVL_0_20			0x0008
#define MBI6334_CONF9_ODVL_0_25			0x000C
//Short Dectection Voltage Level
#define MBI6334_CONF9_SDVL_3			0x0000
#define MBI6334_CONF9_SDVL_6			0x0001
#define MBI6334_CONF9_SDVL_9			0x0002
#define MBI6334_CONF9_SDVL_12			0x0003


//0001 1100 0100 0000	Interrupt Disabled
//#define LGD_DEF_CONF9	0x1C40
#define MLK_DEF_CONF9	(MBI6334_CONF9_FBO_ENABLE | MBI6334_CONF9_ODF_ENALBE | MBI6334_CONF9_SDF_ENALBE |\
		MBI6334_CONF9_EDUN_3 | MBI6334_CONF9_FBOUF_1 | MBI6334_CONF9_LEDEM_DISABLE | MBI6334_CONF9_INT_ENABLE |\
		MBI6334_CONF9_ODVL_0_10 | MBI6334_CONF9_SDVL_3)

//Configuration 10
//#define LGD_DEF_CONF10	0x0000
#define MLK_DEF_CONF10	0xE002

//Configuration 11
//#define LGD_DEF_CONF11	0x00FF
#define MLK_DEF_CONF11	0x3396

//Configuration 12
//#define LGD_DEF_CONF12	0x181B
#define MLK_DEF_CONF12	0x1810

//Configuration 13
//#define LGD_DEF_CONF13	0x0011
#define MLK_DEF_CONF13	0x0211

//Configuration 14
//#define LGD_DEF_CONF14	0x0002
#define MLK_DEF_CONF14	0x0001

//Configuration 15
#define MLK_DEF_CONF15	0x0000

//Configuration 16
//#define LGD_DEF_CONF16	0x0000
#define MLK_DEF_CONF16	0xA03F


#define MBI6334_MAX_BRIGHT	0x0FFF
#define MBI6334_REGADDR_SCAN1_CH64	0x0020
#define MBI6334_REGADDR_SCAN1_CH56	(MBI6334_REGADDR_SCAN1_CH64+8)
#define MBI6334_REGADDR_SCAN2_CH64	0x0060
#define MBI6334_REGADDR_SCAN2_CH56	(MBI6334_REGADDR_SCAN2_CH64+8)
#define MBI6334_REGADDR_SCAN3_CH64	0x00A0
#define MBI6334_REGADDR_SCAN3_CH56	(MBI6334_REGADDR_SCAN3_CH64+8)

#define MBI6334_FRAME_END_ADDR		0x001F
#define MBI6334_FRAME_END_DATA		0x0001

typedef struct {
	uint16_t addr;
	uint16_t data;
}lgd_config_t;


#define MLK_SPI_CS_HIGH()   (CS_GPIO_Port->BSRR = CS_Pin)
#if HW_STMF405
#define MLK_SPI_CS_LOW()	(CS_GPIO_Port->BSRR = CS_Pin << 16)
#else
#define MLK_SPI_CS_LOW()	(CS_GPIO_Port->BRR = CS_Pin)
#endif

typedef struct {
	GPIO_TypeDef*	port;
	uint16_t		pin;
}stm32_gpio_t;

#define SW_SPI_SDI1_PORT	SDI1_GPIO_Port
#define SW_SPI_SDI1_PIN		SDI1_Pin
#define SW_SPI_SDI2_PORT	SDI2_GPIO_Port
#define SW_SPI_SDI2_PIN		SDI2_Pin
#define SW_SPI_SDI3_PORT	SDI3_GPIO_Port
#define SW_SPI_SDI3_PIN		SDI3_Pin
#define SW_SPI_SDI4_PORT	SDI4_GPIO_Port
#define SW_SPI_SDI4_PIN		SDI4_Pin
#define SW_SPI_MISO_PORT
#define SW_SPI_MISO_PIN
#define SW_SPI_SCLK_PORT	SCLK_GPIO_Port
#define SW_SPI_SCLK_PIN		SCLK_Pin
#define SW_SPI_CS_PORT		CS_GPIO_Port
#define SW_SPI_CS_PIN		CS_Pin

typedef struct {
	uint16_t dev;
	uint16_t nData;
	uint16_t reg;
	uint16_t* pData[4];
}continuous_data_t;
typedef struct {
	uint16_t ch[MLK_CHANNEL_NUM];
}mlk_bright_t;
typedef struct {
	uint16_t reg;
	mlk_bright_t part[MLK_PART_NUM];
}mlk_frame_scan_t;

typedef struct {
	uint16_t dev;
	mlk_frame_scan_t scan[MLK_SCAN_NUM];
}mlk_frame_part_t;

typedef struct {
	mlk_frame_part_t ic[MLK_IC_NUM];
}mlk_frame_t;
extern mlk_frame_t FRAME;




void mbi6334_send_data(uint8_t broad, uint8_t devAddr, uint16_t val);
void mbi6334_read_reg3(uint16_t reg);
void MLK_SPI_init(void);
void MLK_SPI_write_single_data(uint16_t dev, uint16_t reg, uint16_t val);
void MLK_SPI_write_continous_data(continuous_data_t *pCont);
void MLK_SPI_write_16bit_4ch(uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4);
void MLK_DISP_config(void);
void MLK_DISP_send_data(uint8_t broad, uint8_t dev, uint16_t val, uint8_t scan);
void MLK_SPI_write_frame_data(void);
#endif /* INC_SPI_H_ */
