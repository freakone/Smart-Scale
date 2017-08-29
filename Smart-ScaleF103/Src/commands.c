#include "commands.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "math.h"
#include "usbd_cdc_if.h"

HX711 hx1;
HX711 hx2;
char command[30];
uint16_t length = 0;
uint8_t status = 0;
uint8_t iCalibration = 87;
uint8_t iTare = 0;
uint16_t temperature = 0;
uint8_t iDFU = 0;

uint32_t startAddress = 0x800FC00;//starting from 1KB before flash ends = 63KB

void writeFlash(void)
{

    HAL_FLASH_Unlock();
    FLASH_PageErase(startAddress);
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, startAddress, iCalibration);
    HAL_FLASH_Lock();
}

void readFlash(void)
{
	iCalibration = *(uint16_t *)(startAddress);
}

void Move_Array(int* arr, int n)
{
	for(int i = 1; i < n; i++)
	{
		arr[i-1] = arr[i];
	}
}

float mean(int m, int a[]) {
    int sum=0, i;
    for(i=0; i<m; i++)
        sum+=a[i];
    return((float)sum/m);
}

float median(int n, int* src) {
    float temp;
    int x[n];
    memcpy(x, src, sizeof x);
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[n/2];
    }
}

void HX711_Process_Values()
{

	if(hx1.readingA < 0)
	{
		hx1.readingA = 0;
	}

	if(hx1.readingB < 0)
	{
		hx1.readingB = 0;
	}

	if(hx2.readingA < 0)
	{
		hx2.readingA = 0;
	}

	if(hx2.readingB < 0)
	{
		hx2.readingB = 0;
	}

	Move_Array(hx1.historyA, FLT);
	Move_Array(hx1.historyB, FLT);
	Move_Array(hx2.historyA, FLT);
	Move_Array(hx2.historyB, FLT);
	hx1.historyA[FLT-1] = hx1.readingA;
	hx1.historyB[FLT-1] = hx1.readingB;
	hx2.historyA[FLT-1] = hx2.readingA;
	hx2.historyB[FLT-1] = hx2.readingB;

	hx1.valueA = median(FLT, hx1.historyA);
	hx1.valueB = median(FLT, hx1.historyB);
	hx2.valueA = median(FLT, hx2.historyA);
	hx2.valueB = median(FLT, hx2.historyB);

	if(hx1.valueA < 6)
	{
		hx1.valueA = 0;
	}

	if(hx1.valueB < 6)
	{
		hx1.valueB = 0;
	}

	if(hx2.valueA < 6)
	{
		hx2.valueA = 0;
	}

	if(hx2.valueB < 6)
	{
		hx2.valueB = 0;
	}
}

void Commands_BufferHandle(uint8_t* Buf, uint32_t *Len)
{
	if (status == 0 && Buf[0] == '|')
	{
	  status = 1;
	  length = 0;
	}

	if (status == 1)
	{
	  for (int i = 0; i < *Len; i++)
	  {
		  command[length] = Buf[i];
		  length++;
	  }
	}

	if (status == 1 && Buf[*Len-1] == '\n')
	{
		status = 0;
		Commands_Parse(command, length);
	}

}

uint8_t _cmd_check(char* buf, uint8_t bufsize, char* cmd, uint8_t cmdsize)
{
	if(cmdsize >= bufsize)
	{
		return 0;
	}

	for(int i = 1; i <= cmdsize; i++)
	{
		if(buf[i] != cmd[i-1])
		{
			return 0;
		}
	}

	return 1;
}

void Commands_Parse(char* buf, uint8_t len)
{
	uint8_t msg[30];
	msg[0] = '|';
	uint16_t offset = 1;

	if(_cmd_check(buf, len, "id", 2))
	{
	  unsigned int *id = (unsigned int *)0x1FFFF7E8;
	  for( int i = 0; i < 3; i++)
	  {
		  offset += sprintf(&msg[offset], "%08X", id[i]);
	  }
	  msg[offset++] = '\n';
	  CDC_Transmit_FS(msg, offset);
	}
	else if(_cmd_check(buf, len, "val", 3))
	{
	  offset += sprintf(&msg[offset], ":%03d", hx1.valueA);
	  offset += sprintf(&msg[offset], ":%03d", hx1.valueB);
	  offset += sprintf(&msg[offset], ":%03d", hx2.valueA);
	  offset += sprintf(&msg[offset], ":%03d", hx2.valueB);
//	  offset += sprintf(&msg[offset], ":%03d", temperature);

	  msg[offset++] = '\n';
	  CDC_Transmit_FS(msg, offset);
	} else if(_cmd_check(buf, len, "cal", 3))
	{

	  char num[] = {buf[4], buf[5]};
	  int param = strtol(num, NULL, 10);

	  if(param > 30 && param < 100)
	  {
		  iCalibration = param;
		  writeFlash();
		  memcpy(&msg[offset], "OK", 2);
	  }
	  else
	  {
		  memcpy(&msg[offset], "ER", 2);
	  }
	  offset += 2;
	  msg[offset++] = '\n';

	  CDC_Transmit_FS(msg, offset);
	} else if(_cmd_check(buf, len, "tare", 4))
	{
	  iTare = 1;
	  memcpy(&msg[offset], "OK", 2);
	  offset += 2;
	  msg[offset++] = '\n';

	  CDC_Transmit_FS(msg, offset);
	}
	else if(_cmd_check(buf, len, "dfu", 3))
	{
		iDFU = 1;
	}

}
