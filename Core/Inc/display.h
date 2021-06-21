/*
 * display.h
 *
 *  Created on: Jun 7, 2021
 *      Author: netbugger
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#define DISP_WIDTH	48
#define DISP_HEIGHT	42

#define DISP_BRIGHT_UPSCALE	1	//16

extern uint16_t DISPLAY[DISP_HEIGHT][DISP_WIDTH];

void DISP_conv_to_FRAME(void);
void TCON_conv_to_DISPLAY(void);

#endif /* INC_DISPLAY_H_ */
