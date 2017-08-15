/*
 * commands.h
 *
 *  Created on: Aug 9, 2016
 *      Author: kamil
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "hx711.h"

extern HX711 hx1;
extern HX711 hx2;
extern uint8_t iCalibration;
extern uint8_t iTare;
extern uint16_t temperature;
extern uint8_t iDFU;
void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len);
void Commands_Parse(char* buf, uint8_t len);
uint8_t _cmd_check(char* buf, uint8_t bufsize, char* cmd, uint8_t cmdsize);
void Move_Array(int* arr, int n);
void writeFlash(void);
void readFlash(void);
void HX711_Process_Values();

#endif /* COMMANDS_H_ */
