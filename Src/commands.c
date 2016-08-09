#include "commands.h"

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

	  uint8_t msg[30];
	  msg[0] = '|';
	  uint16_t offset = 1;

	  if(command[1] == 'i' && command[2] == 'd')
	  {
		  unsigned long *id = (unsigned long *)0x1FFFF7AC;
		  for( int i = 0; i < 3; i++)
		  {
			  offset += sprintf(&msg[offset], "%08X", id[i]);
		  }
		  msg[offset++] = '\n';
		  CDC_Transmit_FS(msg, offset);
	  }
	  else if(command[1] == 'v' && command[2] == 'a' && command[3] == 'l')
	  {
		  srand(HAL_GetTick());
		  for( int i = 0; i < 4; i++)
		  {
			  offset += sprintf(&msg[offset], ":%03X", rand() % 1500);
		  }
		  msg[offset++] = '\n';
		  CDC_Transmit_FS(msg, offset);
	  } else if(command[1] == 'c' && command[2] == 'a' &&
			  command[3] == 'l' && command[4] >= '0' && command[4] <= '9')
	  {
		  memcpy(&msg[offset], "OK", 2);
		  offset += 2;
		  msg[offset++] = '\n';

		  CDC_Transmit_FS(msg, offset);
	  }
	}

}
