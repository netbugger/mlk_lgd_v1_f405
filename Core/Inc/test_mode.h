/*
 * config.h
 *
 *  Created on: 2021. 3. 22.
 *      Author: netbugger
 */
#ifndef INC_TEST_MODE_H_
#define INC_TEST_MODE_H_

#include "main.h"

#define USER_DATA_BASE_ADDR	((uint32_t)0x080E0000)
#define USER_DATA_END_ADDR	((uint32_t)0x08100000)

typedef uint32_t config_t;
extern config_t gConfig;

#define HEADER_LOAD	'L'
#define HEADER_SET	'S'
#define HEADER_RESULT	'R'

// Function Prototype
void load_config(void);
int save_config(void);
int erase_config(void);
void TESTMODE_execute(void);
void TESTMODE_send_data(uint8_t header, uint8_t payload);

#endif /* INC_TEST_MODE_H_ */
