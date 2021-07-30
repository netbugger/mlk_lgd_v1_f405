/*
 * config.h
 *
 *  Created on: 2021. 3. 22.
 *      Author: netbugger
 */
#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

#define USER_DATA_BASE_ADDR	((uint32_t)0x080E0000)
#define USER_DATA_END_ADDR	((uint32_t)0x08100000)

typedef uint32_t config_t;
extern config_t gConfig;

// Function Prototype
void load_config(void);
int save_config(void);
int erase_config(void);

#endif /* INC_CONFIG_H_ */
