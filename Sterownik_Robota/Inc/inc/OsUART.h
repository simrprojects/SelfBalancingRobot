/**\file	OsUART.h
 * \brief   Moduł dostarcza podstawowej funkcjonalności do obsłógi portu komunikacyjnego
 * UART ze wsparciem systemu operacyjnego do synchronizacji
 */
#ifndef OSUART_H_
#define OSUART_H_

#include "usart.h"

typedef void* OsUARTHandler;

int OsUART_Init(OsUARTHandler* h,UART_HandleTypeDef *uart);
int OsUART_Receive(OsUARTHandler h,char *pData,int timeout);
int OsUART_Transmit(OsUARTHandler h,uint8_t *pData,int dataSize,int timeout);
int OsUART_FlushRx(OsUARTHandler h);
#endif
