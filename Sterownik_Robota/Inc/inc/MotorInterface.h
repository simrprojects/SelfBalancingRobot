/*
 * MotorInterface.h
 *
 *  Created on: 03.04.2018
 *      Author: Przemek
 */

#ifndef INC_MOTORINTERFACE_H_
#define INC_MOTORINTERFACE_H_

#include "CAN2UART.h"

typedef void* tMotorInterfaceHandler;
typedef enum{eInactiveMode=0,eActiveMode}tMotorInterfaceMode;
typedef struct{
	int canId;
	int numPolePairs;
	int reversMode;/**<tryb pracy odwr�conej*/
	tCAN2UARTHandle c2u;
}tMotorInterfaceConfig;

typedef struct{
	float voltage;
	float current;
	float rpm;
	float angle;
}tMotorMeasuremenets;


int MotorInterface_Init(tMotorInterfaceHandler *h,tMotorInterfaceConfig *cfg);
int MotorInterface_SetMode(tMotorInterfaceHandler h,tMotorInterfaceMode mode);
int MotorInterface_UpdateControl(tMotorInterfaceHandler h,int ctrl);
int MotorInterface_GetSpeed(tMotorInterfaceHandler h,float *rpm);
int MotorInterface_GetPosition(tMotorInterfaceHandler h,float *angle);
int MotorInterface_GetCurrent(tMotorInterfaceHandler h,float *current);
int MotorInterface_GetVoltage(tMotorInterfaceHandler h,float *voltage);
int MotorInterface_GetMode(tMotorInterfaceHandler h,tMotorInterfaceMode *mode);
void MotorInterface_NewMotorState(tMotorInterfaceHandler h,tMotorInterfaceMode mode);
tMotorMeasuremenets* MotorInterface_GetMeasurements(tMotorInterfaceHandler h);


#endif /* INC_MOTORINTERFACE_H_ */
