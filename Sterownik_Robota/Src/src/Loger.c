/**
  ******************************************************************************
  * @file	 Loger.c
  * @author  Przemek
  * @version V1.0.0
  * @date    04.04.2018
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Loger.h"
#include "stdio.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	tParamDescriptor *paramsArray;
	tLogerStreamerDriver driver;
	tLogDescriptor descriptor;
	int maxNumberOfParams;
	int paramsCounter;
	int period;
	xTaskHandle task;
	xQueueHandle requestQueue;
}tLoger;
/* Private define ------------------------------------------------------------*/
#define HLOGER()	((tLoger*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Loger_Task(tLogerHandler h);
void Loger_Error(tLogerHandler h,char* description);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje instację logera, którego zadaniem jest zbieranie i konwersja danych pomiarowych
  * i przekierowanie ich do jednego z kilku typów nośników/celów
  * @param[in]  None
  * @retval None
  */
int Loger_Create(tLogerHandler *h,tLogerCfg *cfg,tLogerStreamerDriver *drv){
	tLoger *log;
	//alokuje pamięc
	log = pvPortMalloc(sizeof(tLoger));
	//inicjuje parametry wewnętrzne
	log->maxNumberOfParams=cfg->maxNumberOfParams;
	log->paramsCounter = 0;
	log->period = cfg->dtms;
	log->paramsArray = pvPortMalloc(sizeof(tParamDescriptor)*cfg->maxNumberOfParams);
	log->requestQueue=xQueueCreate(10,sizeof(tLogerRequest));
	memcpy(&log->driver,drv,sizeof(tLogerStreamerDriver));
	//inicjuje wątek
	xTaskCreate(Loger_Task,"loger",256,log,1,&log->task);
	*h = log;
	return 0;
}
/**
  * @brief Funkcja dodaje jeden parametr do logowania
  * @param[in]  None
  * @retval None
  */
int Loger_AddParams(tLogerHandler h,void* paramRef,char* paramName,tLogerParamType paramType){
	//sprawdzam czy jest jeszcze wolne miejsce
	if(HLOGER()->maxNumberOfParams>HLOGER()->paramsCounter){
		int i = HLOGER()->paramsCounter;
		HLOGER()->paramsArray[i].src = paramRef;
		HLOGER()->paramsArray[i].name = paramName;
		HLOGER()->paramsArray[i].type = paramType;
		HLOGER()->paramsCounter++;
	}else{
		Loger_Error(h,"Brak wolnych miejsc na nowy parametr");
		return 1;//brak wolnych miejsc
	}
	return 0;
}
/**
  * @brief  Funkcja otwiera sesję rejestrującą pomiary
  * @param[in]  None
  * @retval None
  */
int Loger_OpenSesion(tLogerHandler h){
	tLogerRequest req;
	req = eLR_OpenSesjon;
	if(xQueueSend(HLOGER()->requestQueue,&req,20)==pdTRUE){
			return 0;
	}else{
			return 1;
	}
}
/**
  * @brief  Funkcja zamyka sesję logującą pomiary
  * @param[in]  None
  * @retval None
  */
int Loger_CloseSesion(tLogerHandler h){
	tLogerRequest req;
	req = eLR_CloseSesjon;
	if(xQueueSend(HLOGER()->requestQueue,&req,20)==pdTRUE){
			return 0;
	}else{
			return 1;
	}
}
/**
  * @brief  Pomocnicza funkcja realizująca konwersję zmiennej binarnej na postac tekstową
  * @param[in]  None
  * @retval None
  */
int Loger_Convert2Text(char* buf,tLogerParamType type,void* data){
	switch(type){
	case eParamTypeU8:
		return sprintf(buf,"%d; ",*(unsigned char*)data);
	case eParamTypeI8:
		return sprintf(buf,"%d; ",*(signed char*)data);
	case eParamTypeU16:
		return sprintf(buf,"%d; ",*(unsigned short*)data);
	case eParamTypeI16:
		return sprintf(buf,"%d; ",*(signed short*)data);
	case eParamTypeU32:
		return sprintf(buf,"%d; ",*(unsigned int*)data);
	case eParamTypeI32:
		return sprintf(buf,"%d; ",*(signed int*)data);
	case eParamTypeSGL:
		return sprintf(buf,"%f; ",*(float*)data);
	}
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Wątek przetwarzający rekordy logera
  * @param[in]  None
  * @retval None
  */
void Loger_Task(tLogerHandler h){
	tLoger *log = HLOGER();
	tLogerRequest req;
	int readyToLog=0;
	int period=100;
	//
	while(1){
		//czekam na rozkazy
		if(xQueueReceive(log->requestQueue,&req,period)==pdTRUE){
			switch(req){
				case eLR_OpenSesjon:
					if(readyToLog==0){
						//sesja zamknięta, można ją otworzyć
						if(log->paramsCounter>0){
							//istnieją dane do logowania, otwieram sesję
							log->descriptor.numberOfParams = log->paramsCounter;
							log->descriptor.paramsArray = log->paramsArray;
							log->descriptor.logerStreamerHandler=log->driver.logerStreamerHandler;
							log->driver.open(&log->descriptor);
							//sesja otwarta pomyślnie
							readyToLog = 1;
							//ustawiam okres logowania
							period = log->period;
						}else{
							//brak danych do logowania
						}
					}else{
						//sesja już jest otwarta
					}
					break;
				case eLR_CloseSesjon:
					readyToLog = 0;
					//zamykam sesję
					log->driver.close(&log->descriptor);
					//ustawiam okres logowania
					period = 100;
					break;
			}
		}else{
			//nie odebrano żadnych rozkazów
			if(readyToLog){
				//sesja otwara, zapisuje dane
				log->driver.update(&log->descriptor);
			}
		}
	}
}
__weak void Loger_Error(tLogerHandler h,char* description){

}

