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
		Commands_Parse(command, length);
	}

}

uint8_t _cmd_check(uint8_t* buf, uint8_t bufsize, uint8_t* cmd, uint8_t cmdsize)
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

void Commands_Parse(uint8_t* buf, uint8_t len)
{
	uint8_t msg[30];
	msg[0] = '|';
	uint16_t offset = 1;

	if(_cmd_check(buf, len, "id", 2))
	{
	  unsigned long *id = (unsigned long *)0x1FFFF7AC;
	  for( int i = 0; i < 3; i++)
	  {
		  offset += sprintf(&msg[offset], "%08X", id[i]);
	  }
	  msg[offset++] = '\n';
	  CDC_Transmit_FS(msg, offset);
	}
	else if(_cmd_check(buf, len, "val", 3))
	{
	  srand(HAL_GetTick());
	  for( int i = 0; i < 4; i++)
	  {
		  offset += sprintf(&msg[offset], ":%03X", rand() % 1500);
	  }
	  msg[offset++] = '\n';
	  CDC_Transmit_FS(msg, offset);
	} else if(_cmd_check(buf, len, "cal", 3))
	{
	  memcpy(&msg[offset], "OK", 2);
	  offset += 2;
	  msg[offset++] = '\n';

	  CDC_Transmit_FS(msg, offset);
	}

}
