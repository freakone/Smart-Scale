/*
 * commands.h
 *
 *  Created on: Aug 9, 2016
 *      Author: kamil
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "hx711.h"
#include "math.h"

extern HX711 hx1;
extern HX711 hx2;
extern uint8_t iCalibration;
extern uint8_t iTare;
extern uint16_t temperature;
extern uint8_t iDFU;
void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len);
void Commands_Parse(uint8_t* buf, uint8_t len);
uint8_t _cmd_check(uint8_t* buf, uint8_t bufsize, int8_t* cmd, uint8_t cmdsize);
void Move_Array(int* arr, int n);
void writeFlash(void);
void readFlash(void);

#endif /* COMMANDS_H_ */
