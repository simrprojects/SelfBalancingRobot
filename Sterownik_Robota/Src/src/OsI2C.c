/**
  ******************************************************************************
  * @file	 OSI2c.h
  * @author  Przemek
  * @version V1.0.0
  * @date    06.04.2018
  * @brief   Moduł interfejsu I2C ze wsparciem OS
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "OsI2C.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	 I2C_HandleTypeDef* hi2c;
	 SemaphoreHandle_t mutex;
	 xQueueHandle message;
	 int ready;
 }tI2CComPort;
 
 typedef struct{
	 unsigned char address;
	 tI2CComPort * scp;
 }tI2CComChannel;
/* Private define ------------------------------------------------------------*/
#define HI2C()	((tI2CComPort*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 tI2CComPort i2cComPort[3];
 
 extern I2C_HandleTypeDef hi2c1;
 extern I2C_HandleTypeDef hi2c2;
 extern I2C_HandleTypeDef hi2c3;
/* Private function prototypes -----------------------------------------------*/
 int OSI2CGetIdByRef(I2C_HandleTypeDef* hspi);
 int OSI2CTakeComPort(tI2CComPort * scc, int timeout);
 void OSI2CReleaseComPort(tI2CComPort * scc);
 int OSI2C_GetID(I2C_HandleTypeDef* hi2c);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje warstwę softwereową portu komunikacyjnego I2C
  * @param[in]  None
  * @retval None
  */
int  OSI2C_Init(I2CComPortHandle * scp,I2C_HandleTypeDef* hi2c){
	int id = OSI2CGetIdByRef(hi2c);
	if(id<0){
		return -1;//błąd ID
	}
	i2cComPort[id].mutex=xSemaphoreCreateMutex();
	//inicjuje wiadomosc do komunikacji sprzetu z funkcjami wymiany danych
	i2cComPort[id].message= xQueueCreate(5,4);
	i2cComPort[id].ready=1;
	i2cComPort[id].hi2c=hi2c;
	*scp=&i2cComPort[id];
	return 0;
}
/**
  * @brief  Funkcja odczytuje dane z zewnetrznego czujnika/pamieci gdzie rozmiar pola adresowego wynosi 1B
  * @param[in]  h: uchwyt do obiektu komunikacyjnego i2c
  * @param[in]  devAddress: adres urządzenia na magistrali
  * @param[in]  memAddress: adres wewnętrzego rejestru urzadzenia
  * @param[in]  data: wskaxnik do bufora gdzie wczytane zostaną dane
  * @param[in]  size: rozmiar danych do odczytania
  * @param[in]  timeout: maksymalny czas oczekiwania
  * @retval 0 - brak błędów
  */
int OSI2C_Mem1Read(I2CComPortHandle h,unsigned char devAddress,unsigned char memAddress,void* data,int size,int timeout){
	HAL_StatusTypeDef ret;
	int msg;
	if(OSI2CTakeComPort(h,timeout)==0){
		//otrzymałem dostęp do sprzetu, odczytuje dane
		ret=HAL_I2C_Mem_Read_DMA(HI2C()->hi2c,devAddress,memAddress,1, data, size);
		//czekam na raport zwrotny
		if(ret==HAL_OK){
			if(xQueueReceive(HI2C()->message,&msg,timeout)==pdTRUE){
				//odebrałem wiadomośc
				//zwalniam sprzet
				OSI2CReleaseComPort(HI2C());
				return msg;
			}else{
				//nie doczekano się na odpowiedź zwrotnę
				return -1;
			}
		}else{
			//blad na etapie wysyBania danych
			OSI2CReleaseComPort(HI2C());
			return HAL_I2C_GetError(HI2C()->hi2c);
		}
	}else{
	 //przekroczono timeout
	 return -1;
	}
}
/**
  * @brief  Funkcja zapisuje dane do zewnetrznego czujnika/pamieci gdzie rozmiar pola adresowego wynosi 1B
  * @param[in]  h: uchwyt do obiektu komunikacyjnego i2c
  * @param[in]  devAddress: adres urządzenia na magistrali
  * @param[in]  memAddress: adres wewnętrzego rejestru urzadzenia
  * @param[in]  data: wskaxnik do bufora skąd wczytywane będą dane
  * @param[in]  size: rozmiar danych do wysłania
  * @param[in]  timeout: maksymalny czas oczekiwania
  * @retval 0 - brak błędów
  */
int OSI2C_Mem1Write(I2CComPortHandle h,unsigned char devAddress,unsigned char memAddress,void* data,int size,int timeout){
	HAL_StatusTypeDef ret;
	int msg;
	if(OSI2CTakeComPort(h,timeout)==0){
		//otrzymałem dostęp do sprzetu, odczytuje dane
		ret=HAL_I2C_Mem_Write_DMA(HI2C()->hi2c,devAddress,memAddress,1, data, size);
		//czekam na raport zwrotny
		if(ret==HAL_OK){
			if(xQueueReceive(HI2C()->message,&msg,timeout)==pdTRUE){
				//odebrałem wiadomośc
				//zwalniam sprzet
				OSI2CReleaseComPort(HI2C());
				return msg;
			}else{
				//nie doczekano się na odpowiedź zwrotnę
				return -1;
			}
		}else{
			//blad na etapie wysyBania danych
			OSI2CReleaseComPort(HI2C());
			return HAL_I2C_GetError(HI2C()->hi2c);
		}
	}else{
	 //przekroczono timeout
	 return -1;
	}
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja oczekuje na dostęp do sprzetu
  * @param[in]  None
  * @retval 0 - dostęp uzyskany
  */
int OSI2CTakeComPort(tI2CComPort * scc, int timeout){
	if(xSemaphoreTake(scc->mutex,timeout)==pdTRUE){
		return 0;
	}else{
		return 1;
	}
}
/**
  * @brief  Funkcja zwalnia dostęp do sprzetu
  * @param[in]  None
  * @retval none
  */
void OSI2CReleaseComPort(tI2CComPort * scc){
	xSemaphoreGive(scc->mutex);
}
/**
  * @brief  Funkcja zwraca ID przypisane do danej instancji portu I2C
  * @param[in]  None
  * @retval None
  */
int OSI2CGetIdByRef(I2C_HandleTypeDef* hspi){
	if(hspi->Instance==I2C1){
		return 0;
	}else if(hspi->Instance==I2C2){
		return 1;
	}else{
		return 2;
	}
}
/**
  * @brief  Funkcja callBack
  * @param[in]  None
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	int msg=0;
	BaseType_t xHigherPriorityTaskWoken;

	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	/* Post the byte. */
	xQueueSendFromISR(i2cComPort[OSI2CGetIdByRef(hi2c)].message , &msg, &xHigherPriorityTaskWoken );

	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken )
	{
		/* Actual macro used here is port specific. */
		taskYIELD();
	}
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	int msg=0;
	BaseType_t xHigherPriorityTaskWoken;

	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	/* Post the byte. */
	xQueueSendFromISR(i2cComPort[OSI2CGetIdByRef(hi2c)].message , &msg, &xHigherPriorityTaskWoken );

	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken )
	{
		/* Actual macro used here is port specific. */
		taskYIELD();
	}
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	int msg=HAL_I2C_GetError(hi2c);
	BaseType_t xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	/* Post the byte. */
	xQueueSendFromISR(i2cComPort[OSI2CGetIdByRef(hi2c)].message , &msg, &xHigherPriorityTaskWoken );

	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken )
	{
		/* Actual macro used here is port specific. */
		taskYIELD();
	}
}

