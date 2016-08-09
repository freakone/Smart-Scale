/*
 * commands.h
 *
 *  Created on: Aug 9, 2016
 *      Author: kamil
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "stm32f3xx_hal.h"

uint8_t command[30];
uint16_t length = 0;
uint8_t status = 0;
void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len);
void Commands_Parse(uint8_t* buf, uint8_t len);
uint8_t _cmd_check(uint8_t* buf, uint8_t bufsize, uint8_t* cmd, uint8_t cmdsize);

#endif /* COMMANDS_H_ */
