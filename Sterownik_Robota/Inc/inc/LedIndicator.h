/*
 * LedIndicator.h
 *
 *  Created on: 06.04.2018
 *      Author: Przemek
 */

#ifndef INC_LEDINDICATOR_H_
#define INC_LEDINDICATOR_H_

typedef void* tLedIndictorHandler;

typedef enum{
	eLedIndicator_MotorInit=0,/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
	eLedIndicator_ReadyToWork,/*<tryb aktywnej pracy silników bez stabilizacji robota*/
	eLedIndicator_RobotStabilisation,/*<tryb pełnej stabilizacji robota*/
	eLedIndicator_FaultState,/*<awaria podsystemu czujników lub silników*/
}tLedIndicatorState;

int LedIndicator_Init(tLedIndictorHandler *h);
int LedIndicator_SetState(tLedIndictorHandler h,tLedIndicatorState state);

#endif /* INC_LEDINDICATOR_H_ */
