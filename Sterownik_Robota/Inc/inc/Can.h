#ifndef CAN_H_
#define CAN_H_

#include "stm32f2xx_can.h"

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
	unsigned char canMode;
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
	tFilterConfig filter[2];
	tFilterConfig mask[2];
	tFilterModeSelection mode;
	tFilterSize scale;
	unsigned char enable;
}tCanFilter;

void Can_Init(int ch,int rxSize,int txSize);
int Can_ComInit(tCanInit * initStruct,int ch);
int Can_FilterInit(tCanFilter *filter,int ch,int* filterId);
int Can_ReceiveMessage(int ch,CanRxMsg *rx,int timeout);
int Can_SendMessage(int ch,CanTxMsg *tx,int timeout);
int Can_GetRxQueueMsgNumber(int ch);
int Can_Enable(int ch,int enable);

#endif
