/*
 * komendy_at.c
 *
 *  Created on: 06.08.2017
 *      Author: Mariusz
 */

#include "string.h"
#include "stdlib.h"

#include "DISCOVERY/uart.h"
#include "DISCOVERY/komendy_at.h"



#define AT_CNT 4

TATCMD polecenia_at[AT_CNT] = {
	// { at_cmd, wskaznik do funkcji oblusgujace polecenie }
		{"AT", at_service},
		{"ATI", ati_service},
		{"ENTER", enter_service},
		{"AT+PID1_P", at_pid1_p_service}
};


void parse_uart_data(char *pBuf)
{
	char *cmd_wsk;
	char *reszta;

	uint8_t i, len;
	int8_t (*_at_srv)(uint8_t inout, char *params);

	if( strpbrk(pBuf, "=?") ) // przeszukuje string w poszukiwaniu znaku '=' lub '?'
	// czyli napewno polecenie AT+CMD? lub AT+CMD=parametry
	{
		if(strpbrk(pBuf, "?"))
		// polecenie w postaci AT+CMD?
		{
			cmd_wsk = strtok_r(pBuf, "?", &reszta); // tutaj wyluskujemy polecenie AT (w cmd_wsk bedzie AT+CMD ) bez znaku zapytania
			len = strlen(cmd_wsk);

			for(i=0; i<AT_CNT;i++)
			{
				if(0 == strncasecmp(cmd_wsk,polecenia_at[i].polecenie_at,len)&&(len == strlen(polecenia_at[i].polecenie_at)))
				{
					if(polecenia_at[i].at_service)
					{
						_at_srv = polecenia_at[i].at_service;
						if(_at_srv ) _at_srv(0,reszta);
						break;
					}
				}
			}
		}
		else
		// polecenie w postaci AT+CMD = parametry
		{
			cmd_wsk = strtok_r(pBuf, "=", &reszta); // tutaj wyluskujemy polecenie AT (w cmd_wsk bedzie AT+CMD ) bez znaku zapytania
			len = strlen(cmd_wsk);

			for(i=0; i<AT_CNT;i++)
			{
				if(0 == strncasecmp(cmd_wsk,polecenia_at[i].polecenie_at,len)&&(len == strlen(polecenia_at[i].polecenie_at)))
				{
					if(polecenia_at[i].at_service)
					{
						_at_srv = polecenia_at[i].at_service;
						if(_at_srv) _at_srv(1,reszta);
						break;
					}
				}
			}
		}
	}
	else
	// polecenie AT bez parametrow
	{
		if( 0==pBuf[0] ) uart_putstring("\r\n");
		else
		{
			len = strlen(pBuf);
			for(i=0; i<AT_CNT;i++)
			{
				if((0 == strncasecmp(pBuf,polecenia_at[i].polecenie_at,len))&&(len == strlen(polecenia_at[i].polecenie_at)))
				{
					if(polecenia_at[i].at_service)
					{
						_at_srv = polecenia_at[i].at_service;
						if(_at_srv) _at_srv(2,0);
					}
					break;
				}
			}
		}
	}
	if( AT_CNT == i ) uart_putstring("ERROR - unknow cmd \r\n");
}


//int8_t at_..._service(uint8_t inout, char *params)
//{
//	if(inout == 0)
//	{
//	}
//	if(inout == 1)
//	{
//	}
//	if(inout == 2)
//	{
//	}
//	return 0;
//}


int8_t at_service(uint8_t inout, char *params)
{
	if(inout == 0)
	{
		uart_putstring("Zglasza sie funkcja: at ??? \r\n");
	}
	if(inout == 1)
	{
		uart_putstring("Zglasza sie funkcja: at === ");
		uart_putstring(params);
		uart_putstring("\r\n");

	}
	if(inout == 2)
	{
		uart_putstring("OK\r\n");
	}
		return 0;
}
int8_t ati_service(uint8_t inout, char *params)
{
	if(inout == 0)
	{
		uart_putstring("Zglasza sie funkcja: ati ??? \r\n");
	}
	if(inout == 1)
	{
		uart_putstring("Zglasza sie funkcja: ati === ");
		uart_putstring(params);
		uart_putstring("\r\n");
	}
	if(inout == 2)
	{
		uart_putstring("STM32F3DISCOVERY v1.00 \r\n");
	}
		return 0;
}
int8_t enter_service(uint8_t inout, char *params)
{
	if(inout == 0)
	{
		uart_putstring("Zglasza sie funkcja: enter ??? \r\n");
	}
	if(inout == 1)
	{
		uart_putstring("Zglasza sie funkcja: enter === ");
		uart_putstring(params);
		uart_putstring("\r\n");
	}
	if(inout == 2)
	{
		uart_putstring("\r\n");
	}
		return 0;
}
int8_t at_pid1_p_service(uint8_t inout, char *params)
{
	if(inout == 0)
	{
		uart_putstring("PID1_P ILE WYNOSI\r\n");
	}
	if(inout == 1)
	{
		uart_putstring("PID1_P ILE PAN SOBIE chce\r\n");
	}
	if(inout == 2)
	{
		uart_putstring("zmienna1=(0-255) \r\n");
	}
	return 0;
}








