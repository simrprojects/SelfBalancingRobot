/*
 * MotorInterfaceUart.h
 *
 *  Created on: 10.12.2018
 *      Author: Przemek
 */

#ifndef INC_MOTORINTERFACEUART_H_
#define INC_MOTORINTERFACEUART_H_

#include "OsUart.h"
#include "MotorInterface.h"

typedef void* tMotorInterfaceUartHandler;
typedef struct{
	UART_HandleTypeDef *uart;
	unsigned int numPolePairs:5;
	unsigned int reversMode:1;/**<tryb pracy odwr�conej*/
}tMotorInterfaceUartConfig;



int MotorInterfaceUart_Init(tMotorInterfaceUartHandler *h,tMotorInterfaceUartConfig *cfg);
int MotorInterfaceUart_SetMode(tMotorInterfaceUartHandler h,tMotorInterfaceMode mode);
int MotorInterfaceUart_UpdateControl(tMotorInterfaceUartHandler h,int ctrl);
int MotorInterfaceUart_GetSpeed(tMotorInterfaceUartHandler h,float *rpm);
int MotorInterfaceUart_GetPosition(tMotorInterfaceUartHandler h,float *angle);
int MotorInterfaceUart_GetCurrent(tMotorInterfaceUartHandler h,float *current);
int MotorInterfaceUart_GetVoltage(tMotorInterfaceUartHandler h,float *voltage);
int MotorInterfaceUart_GetMode(tMotorInterfaceUartHandler h,tMotorInterfaceMode *mode);
void MotorInterfaceUart_NewMotorState(tMotorInterfaceUartHandler h,tMotorInterfaceMode mode);
tMotorMeasuremenets* MotorInterfaceUart_GetMeasurements(tMotorInterfaceUartHandler h);

#endif /* INC_MOTORINTERFACEUART_H_ */
