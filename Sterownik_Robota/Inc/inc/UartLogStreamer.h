/*
 * UartLogStreamer.h
 *
 *  Created on: 04.04.2018
 *      Author: Przemek
 */

#ifndef INC_UARTLOGSTREAMER_H_
#define INC_UARTLOGSTREAMER_H_

#include "Loger.h"
#include "OsUart.h"

typedef struct{
	OsUARTHandler huart;
	int bufferSize;
	void* bufferPtr;
}tUartStreamConfig;

tLogerStreamerDriver* UartLogStreamer_Init(tUartStreamConfig *cfg);

#endif /* INC_UARTLOGSTREAMER_H_ */
