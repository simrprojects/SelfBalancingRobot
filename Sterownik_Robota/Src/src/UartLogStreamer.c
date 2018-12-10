/**
  ******************************************************************************
  * @file	 UartLogStreamer.c
  * @author  Przemek
  * @version V1.0.0
  * @date    04.04.2018
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <OsUART.h>
#include "FreeRTOS.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "UartLogStreamer.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	OsUARTHandler huart;
	char* buffer;
	int bufferSize;
}tUartLogStreamer;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
int UartLogStreamer_OpenLogerSesion(tLogDescriptor* descriptor);
int UartLogStreamer_tCloseLogerSesion(tLogDescriptor* descriptor);
int UartLogStreamer_tUpdateLogerSesion(tLogDescriptor* descriptor);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief Funkcja inicjuje moduł przesyłający dane z modułu Logera do Uarta
  * @param[in]  None
  * @retval None
  */
tLogerStreamerDriver* UartLogStreamer_Init(tUartStreamConfig *cfg){
	tUartLogStreamer *uart;
	tLogerStreamerDriver *driver;
	//alokuje pamięc
	uart = pvPortMalloc(sizeof(tUartLogStreamer));
	uart->huart=cfg->huart;
	uart->bufferSize = cfg->bufferSize;
	if(cfg->bufferPtr!=0){
		//wykorzystuje zewnętrzny bufor
		uart->buffer = (char*)cfg->bufferPtr;
	}else{
		//alokuje pamięc na bufor
		uart->buffer = pvPortMalloc(cfg->bufferSize);
	}
	//alokuje pamięc na driver
	driver = pvPortMalloc(sizeof(tLogerStreamerDriver));
	//inicjuje driver
	driver->logerStreamerHandler = uart;
	driver->open = UartLogStreamer_OpenLogerSesion;
	driver->close = UartLogStreamer_tCloseLogerSesion;
	driver->update = UartLogStreamer_tUpdateLogerSesion;
	return driver;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja otwierająca sesję logowania
  * @param[in]  None
  * @retval None
  */
int UartLogStreamer_OpenLogerSesion(tLogDescriptor* descriptor){
	return 0;
}
/**
  * @brief  Funkcja zamyka sesję logera
  * @param[in]  None
  * @retval None
  */
int UartLogStreamer_tCloseLogerSesion(tLogDescriptor* descriptor){
	return 0;
}
/**
  * @brief  Funkcja formatuje i wysyła dane na port UART
  * @param[in]  None
  * @retval None
  */
int UartLogStreamer_tUpdateLogerSesion(tLogDescriptor* descriptor){
	tUartLogStreamer *uart=(tUartLogStreamer*)descriptor->logerStreamerHandler;
	int numOfRecords = descriptor->numberOfParams;
	tParamDescriptor *array = descriptor->paramsArray;
	char* buffer = uart->buffer;
	int cnt=0;
	//przetwarzam tablice zmiennych
	for(int i=0;i<numOfRecords;i++){
		//dodaje nazwę parametru
		cnt+= sprintf(&buffer[cnt],"%s: ",array[i].name);
		//dokonuje konwersji zgodnie z typem parametru
		cnt+= Loger_Convert2Text(&buffer[cnt],array[i].type,array[i].src);
		if(cnt>(uart->bufferSize-10)){
			//kończę
			break;
		}
	}
	//dodaje znak końca linii
	cnt+= sprintf(&buffer[cnt],"%s, ","\n\r");
	//po konwersji, wysyłam dane
	return OsUART_Transmit(uart->huart,(unsigned char*)buffer,cnt,0);
}
