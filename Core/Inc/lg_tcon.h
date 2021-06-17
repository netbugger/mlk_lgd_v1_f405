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
#define TCON_FRAME_LEN	1011	//indicator + (24*42) + checksum
//#define TCON_FRAME_LEN	1	//indicator + (24*42) + checksum
#define TCON_DATA_LEN	1008
#define TCON_CHECKSUM_IDX	1009
typedef struct {
	uint32_t idx;
	uint8_t indicator;
	uint8_t data[TCON_HEIGHT][TCON_WIDTH];
	uint8_t checksum;
}tcon_frame_t;




#endif /* INC_LG_TCON_H_ */
