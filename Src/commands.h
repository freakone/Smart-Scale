/*
 * commands.h
 *
 *  Created on: Aug 9, 2016
 *      Author: kamil
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "stm32f3xx_hal.h"
#include "hx711.h"

extern HX711 hx1;
extern HX711 hx2;
extern uint8_t iCalibration;
extern uint8_t iTare;
void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len);
void Commands_Parse(uint8_t* buf, uint8_t len);
uint8_t _cmd_check(uint8_t* buf, uint8_t bufsize, uint8_t* cmd, uint8_t cmdsize);

#endif /* COMMANDS_H_ */
