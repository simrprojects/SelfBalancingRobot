/*
 * CAN2UART.h
 *
 *  Created on: 02.04.2018
 *      Author: Przemek
 */

#ifndef INC_CAN2UART_H_
#define INC_CAN2UART_H_

typedef void* tCAN2UARTHandle;
typedef void* tCAN2UARTChannelRef;

typedef struct{
	int maxNumberOfChannels;
	int canFifoNumber;
}tCAN2UARTConfig;

typedef struct{
	int rxID;
	int txID;
	int txQueueDepth;
	int rxQueueDepth;
}tCAN2UARTChannelCfg;

int CAN2UART_Init(tCAN2UARTHandle *h,tCAN2UARTConfig *cfg);
int CAN2UART_CreateChannel(tCAN2UARTHandle h,tCAN2UARTChannelRef *chRef,tCAN2UARTChannelCfg *cfg);
int CAN2UART_Receive(tCAN2UARTChannelRef h,char* data,int timeout);
int CAN2UART_Transmit(tCAN2UARTChannelRef h,void* data,int dataSize,int timeout);

#endif /* INC_CAN2UART_H_ */
