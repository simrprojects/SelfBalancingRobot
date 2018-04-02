#ifndef OSCAN_H_
#define OSCAN_H_

#include "stm32f7xx_hal.h"

typedef enum{eFilterMaskMode=0,eFilterListMode}tFilterModeSelection;
typedef enum{eStdFilterSize=0,eExtFilterSize}tFilterSize;
typedef enum{eCanSpeed_1Mb=0,
						 eCanSpeed_500kb,
						 eCanSpeed_250kb,
						 eCanSpeed_125kb,
						 eCanSpeed_100kb,
						 eCanSpeed_75kb,
						 eCanSpeed_50kb,
						 eCanSpeed_25kb,
						 eCanSpeed_10kb,
						 eCanSpeed_5kb,
						 eCanSpeed_1kb}tCanSpeedConfig;

typedef struct{
	tCanSpeedConfig canSpeed;
	int rxBufferSize;
}tCanInit;

typedef struct{
	union{
		unsigned int ext;
		unsigned short std;
	}id;
	unsigned char rtr;
	unsigned char ide;
}tFilterConfig;

typedef struct{
	unsigned int id;
	unsigned char dlc;
	unsigned char ide;
	unsigned char rtr;
	unsigned char data[8];
}tCanRxMessage;

typedef struct{
	unsigned int id;
	unsigned char dlc;
	unsigned char ide;
	unsigned char rtr;
	unsigned char data[8];
}tCanTxMessage;

typedef struct{
	tFilterConfig filter[2];
	tFilterConfig mask[2];
	tFilterModeSelection mode;
	tFilterSize scale;
	unsigned char enable;
	unsigned char FIFOSelect;
}tCanFilter;

int OSCan_Init(CAN_HandleTypeDef *can,tCanInit *cfg);
int OSCan_FilterInit(tCanFilter *filter,int* filterId);
int OSCan_ReceiveMessage(tCanRxMessage *rx,int bufferId,int timeout);
int OSCan_SendMessage(tCanTxMessage *tx,int timeout);


int OSCan_TestInLoopBackMode(CAN_HandleTypeDef *can);
#endif
