/**
  ******************************************************************************
  * @file	 LabViewUartStreamer.c
  * @author  Przemek
  * @version V1.0.0
  * @date    12.04.2018
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <OsUART.h>
#include "FreeRTOS.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "LabViewUartStreamer.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	OsUARTHandler huart;
	char* buffer;
	float* data;
	int bufferSize;
	int numOfRecords;
}tLabViewUartStreamer;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
int LabViewUartStreamer_OpenLogerSesion(tLogDescriptor* descriptor);
int LabViewUartStreamer_tCloseLogerSesion(tLogDescriptor* descriptor);
int LabViewUartStreamer_tUpdateLogerSesion(tLogDescriptor* descriptor);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief Funkcja inicjuje moduł przesyłający dane z modułu Logera do Uarta w formacie kompatybilnym z LabView
  * @param[in]  None
  * @retval None
  */
tLogerStreamerDriver* LabViewUartStreamer_Init(tLabViewUartStreamConfig *cfg){
	tLabViewUartStreamer *uart;
	tLogerStreamerDriver *driver;
	//alokuje pamięc
	uart = pvPortMalloc(sizeof(tLabViewUartStreamer));
	uart->huart=cfg->huart;
	uart->bufferSize = cfg->bufferSize;
	uart->numOfRecords = (cfg->bufferSize-7)/4;
	if(cfg->bufferPtr!=0){
		//wykorzystuje zewnętrzny bufor
		uart->buffer = (char*)cfg->bufferPtr;
	}else{
		//alokuje pamięc na bufor
		uart->buffer = pvPortMalloc(cfg->bufferSize);
	}
	//alokuje pamięc na driver
	driver = pvPortMalloc(sizeof(tLogerStreamerDriver));
	//ustawiam wskaźnik na dane
	uart->data = (float*)&uart->buffer[5];
	//inicjuje driver
	driver->logerStreamerHandler = uart;
	driver->open = LabViewUartStreamer_OpenLogerSesion;
	driver->close = LabViewUartStreamer_tCloseLogerSesion;
	driver->update = LabViewUartStreamer_tUpdateLogerSesion;
	return driver;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja otwierająca sesję logowania
  * @param[in]  None
  * @retval None
  */
int LabViewUartStreamer_OpenLogerSesion(tLogDescriptor* descriptor){
	return 0;
}
/**
  * @brief  Funkcja zamyka sesję logera
  * @param[in]  None
  * @retval None
  */
int LabViewUartStreamer_tCloseLogerSesion(tLogDescriptor* descriptor){
	return 0;
}
/**
  * @brief  Funkcja formatuje i wysyła dane na port UART
  * @param[in]  None
  * @retval None
  */
int LabViewUartStreamer_tUpdateLogerSesion(tLogDescriptor* descriptor){
	tLabViewUartStreamer *uart=(tLabViewUartStreamer*)descriptor->logerStreamerHandler;
	int numOfRecords = descriptor->numberOfParams;
	tParamDescriptor *array = descriptor->paramsArray;
	char* buffer = uart->buffer;
	float* data = uart->data;
	float tmp;
	//sprawdzam, czy liczba danych nie jest większa od miejsca w buforze
	if(numOfRecords>uart->bufferSize){
		numOfRecords=uart->bufferSize;
	}
	//formułuję ramkę
	buffer[0]=0x12;
	buffer[1]=0x34;
	*(unsigned short*)&buffer[2]=1+numOfRecords*4;
	buffer[4]=1;//id
	//przetwarzam tablice zmiennych
	for(int i=0;i<numOfRecords;i++){
		//dokonuje konwersji zgodnie z typem parametru
		tmp = Loger_Convert2Float(array[i].type,array[i].src);
		data[i]=tmp;
	}
	//po konwersji, wysyłam dane
	return OsUART_Transmit(uart->huart,(unsigned char*)buffer,numOfRecords*4+5,0);
}


