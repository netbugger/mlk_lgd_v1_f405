/*
 * display.c
 *
 *  Created on: Jun 7, 2021
 *      Author: netbugger
 */

#include "spi.h"
#include "display.h"
#include "lg_tcon.h"
#include "printf.h"
extern tcon_frame_t TCON_FRAME[];
uint16_t DISPLAY[DISP_HEIGHT][DISP_WIDTH] = {
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},

		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0x000, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},

		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},

		{0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0x000, 0x000, 0xfff, 0xfff, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},

		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000},
		{0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000}
	};

void DISP_conv_to_FRAME(void)
{
	int i, j, part, ic;
	int fidx;
	for(ic=0; ic < MLK_IC_NUM; ic++) {
		for(part =0; part < MLK_PART_NUM; part++) {
			// Set SCAN 0
			for(i=0; i<4; i++) {
				for(j=0; j<48;  j++) {
					fidx = 55-((j%12)+(i*12));
					FRAME.ic[ic].scan[0].part[j/12].ch[fidx] = DISPLAY[i+(ic*14)][j];
				}
			}
			i = 4;
			for(j=0; j<48; j++) {
				if(j%12 < 8) {
					fidx = 55-((j%12)+(i*12));
					FRAME.ic[ic].scan[0].part[j/12].ch[fidx] = DISPLAY[i+(ic*14)][j];
				}
				else {
					//fidx = 55 - ((j%12) - 8);
					fidx = 55-((j%12)+((i-5)*12) + 4);
					FRAME.ic[ic].scan[1].part[j/12].ch[fidx] = DISPLAY[i+(ic*14)][j];
				}
			}

			for(i=5; i<9; i++) {
				for (j = 0; j < 48; j++) {
					fidx = 55-((j%12)+((i-5)*12) + 4);
					FRAME.ic[ic].scan[1].part[j / 12].ch[fidx] = DISPLAY[i + (ic * 14)][j];
				}
			}
			i = 9;
			for (j = 0; j < 48; j++) {
				if (j % 12 < 4) {
					fidx = 55-((j%12)+((i-5)*12) + 4);
					FRAME.ic[ic].scan[1].part[j / 12].ch[fidx] = DISPLAY[i + (ic * 14)][j];
				} else {
					fidx = 55-((j%12)+((i-10)*12) + 8);
					FRAME.ic[ic].scan[2].part[j / 12].ch[fidx] = DISPLAY[i + (ic * 14)][j];
				}
			}
			for(i=10; i < 14; i++) {
				for (j = 0; j < 48; j++) {
					fidx = 55-((j%12)+((i-10)*12) + 8);
					FRAME.ic[ic].scan[2].part[j / 12].ch[fidx] = DISPLAY[i + (ic * 14)][j];
				}
			}

		}
	}

}


void TCON_conv_to_DISPLAY(void)
{
	uint16_t max, lmax = 0;
	int h, w;
	for(h = 0; h < DISP_HEIGHT; h++) {
		for(w = 0; w < DISP_WIDTH; w++) {
			if( w < 24 ) {
				DISPLAY[h][w] = ((uint16_t)TCON_FRAME[0].data[(h*24)+w+2])*DISP_BRIGHT_UPSCALE;
			}
			else {
				DISPLAY[h][w] = ((uint16_t)TCON_FRAME[1].data[(h*24)+(w-24)+2])*DISP_BRIGHT_UPSCALE;
			}
#if 0
			max = DISPLAY[h][w];
			if(max > lmax) {
				printf("max=%x\n", max);
				lmax = max;
			}
#endif
		}
	}


}
