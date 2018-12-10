/*
 * LabViewUartStreamer.h
 *
 *  Created on: 12.04.2018
 *      Author: Przemek
 */

#ifndef INC_LABVIEWUARTSTREAMER_H_
#define INC_LABVIEWUARTSTREAMER_H_

#include "Loger.h"
#include "OsUart.h"

typedef struct{
	OsUARTHandler huart;
	int bufferSize;
	void* bufferPtr;
}tLabViewUartStreamConfig;

tLogerStreamerDriver* LabViewUartStreamer_Init(tLabViewUartStreamConfig *cfg);

#endif /* INC_LABVIEWUARTSTREAMER_H_ */
