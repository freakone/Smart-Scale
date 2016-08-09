/*
 * commands.h
 *
 *  Created on: Aug 9, 2016
 *      Author: kamil
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "usb_device.h"

uint8_t command[30];
uint16_t length = 0;
uint8_t status = 0;
void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len);

#endif /* COMMANDS_H_ */
