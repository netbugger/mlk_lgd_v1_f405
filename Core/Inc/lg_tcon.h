/*
 * lg_tcon.h
 *
 *  Created on: 2021. 6. 10.
 *      Author: netbugger
 */

#ifndef INC_LG_TCON_H_
#define INC_LG_TCON_H_

#include "main.h"

#define TCON_WIDTH	24
#define TCON_HEIGHT	48
#define TCON_INDICATOR_VAL		0xAA
#define TCON_FRAME_LEN	1011	//indicator + cmd + (24*42) + checksum
#define TCON_OFFSET_INDICATOR	0
#define TCON_OFFSET_CMD			1
#define TCON_OFFSET_CHKSUM		1010
#define TCON_CH_NUM				2
//#define TCON_FRAME_LEN	1	//indicator + (24*42) + checksum
#define TCON_DATA_LEN	1008
typedef struct {
	uint8_t vsync;
	uint8_t complete;
	SPI_HandleTypeDef *pHspi;
	DMA_HandleTypeDef *pHdma;
	uint8_t data[TCON_FRAME_LEN];
}tcon_frame_t;

void TCON_init(void);


#endif /* INC_LG_TCON_H_ */
