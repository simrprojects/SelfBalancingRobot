/**
  ******************************************************************************
  * @file		 OsUART.c
  * @author  PS
  * @version V1.0.0
  * @date    08.06.2017
  * @brief   Moduł dostarcza podstawowej funkcjonalności do obsłógi portu komunikacyjnego
	* UART ze wsparciem systemu operacyjnego do synchronizacji
	*
	* Zadaniem modułu jest dostarczenie wybranych funkcji modułu HAL tyle, że współpracujacych z OS
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "OsUART.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* Private typedef -----------------------------------------------------------*/
/**\define 
 *\brief definicja aktywuje lub deaktywuje wsparcie dla wielowątkowego wsparcia dla 
 *       dostępu do portu komuniakcyjnego. Należy ustawić 1 gdy więcej niż jeden task będzie 
 *       chciał dostępu do tego zasobu
 */
#define OSUART_REENTRANT	0

typedef struct{
	QueueHandle_t msg;
	QueueHandle_t rx;
	UART_HandleTypeDef *uart;
}tOSUart;


/* Private define ------------------------------------------------------------*/
#define HUART		((tOSUart*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tOSUart osUsartTab[7];
/* Private function prototypes -----------------------------------------------*/
int OsUart_WaitOnMessage(tOSUart *uart,int timeout);
void OsUart_SendMessage(int id,int msg);
int OsUart_GetId(UART_HandleTypeDef *uart);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje podstawowe elementy modułu
	* @param[out] h: wskaźnik do uchwytu do obiektu zwracany przez funkcję
	* @param[in] qspi: wskaźnik do obiektu z biblioteki hal
  * @retval 0 - brak błędów
  */
int OsUART_Init(OsUARTHandler* h,UART_HandleTypeDef *uart){
	int id;
	//odczytuje id
	id = OsUart_GetId(uart);
	if(id==-1){
		//błąd
		return 1;
	}
	//inicjuje kolejkę na wiadomości
	osUsartTab[id].msg = xQueueCreate(1,sizeof(int));
	//inicjuje kolejkę na dane odbiorcze
	osUsartTab[id].rx = xQueueCreate(100,sizeof(char));
	//inicjuje zmienne
	osUsartTab[id].uart = uart;
	//zwracam uchwyt
	*h = &osUsartTab[id];
	//uruchamiam odbiornik
	osUsartTab[id].uart->Instance->CR1|=0x1<<5;
	SET_BIT(uart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
	return 0;
}
/**
  * @brief  Funkcja wywołuję operację odczytu bajta danych
  * @param[in] h: uchwyt do obiektu komunikacyjnego
	* @param[in] pData: wskaźnik do bufora na odbierane dane
	* @param[in] timeout: maksymalny czas oczekiwania na wykonanie operacji
  * @retval 0 - brak błedów
  */
int OsUART_Receive(OsUARTHandler h,char *pData,int timeout){
	if(xQueueReceive(HUART->rx,pData,timeout)==pdTRUE){
		return 0;
	}else{
		return 1;
	}
}
/**
  * @brief  Funkcja wywołuje operację zapisue danych do urzadzenia podłączonego do magistrali
  * @param[in] h: uchwyt do obiektu komunikacyjnego
	* @param[in] pData: wskaźnik do bufora z danymi do wysłania
	* @param[in] timeout: maksymalny czas oczekiwania na wykonanie operacji
  * @retval 0 - brak błedów
  */
int OsUART_Transmit(OsUARTHandler h,uint8_t *pData,int dataSize,int timeout){
	if(timeout==0){
		//HAL_UART_Transmit_DMA(HUART->uart,pData,dataSize);
		//HAL_DMA_Abort(HUART->uart->hdmatx);
		//__HAL_UNLOCK(HUART->uart->hdmatx);
		//HUART->uart->hdmatx->State = HAL_DMA_STATE_READY;
//		CLEAR_BIT(HUART->uart->Instance->CR3, USART_CR3_DMAT);
//		HAL_DMA_Start_IT(HUART->uart->hdmatx, (int)pData, (uint32_t)&HUART->uart->Instance->TDR, dataSize);
//		SET_BIT(HUART->uart->Instance->CR3, USART_CR3_DMAT);
		return 0;
	}else{
		return HAL_UART_Transmit(HUART->uart,pData,dataSize,timeout);
	}
}
/**
  * @brief  Funkcja kasuje zawartośc bufora odbiorczego
  * @@param[in] h: uchwyt do obiektu komunikacyjnego 
  * @retval 0 - brak błedów
  */
int OsUART_FlushRx(OsUARTHandler h){
	xQueueReset(HUART->rx);
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja przypisuje ID dla konkretnego portu UART
  * @param  None
  * @retval None
  */
int OsUart_GetId(UART_HandleTypeDef *uart){
	if(uart->Instance == USART2){
		return 1;
	}else if(uart->Instance == USART6){
		return 5;
	}else if(uart->Instance == USART1){
		return 0;
	}else{
		return -1;
	}
}
/**
  * @brief  Funkcja przerwania od UARTA2
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	char cIn;

	if(USART1->ISR&(0x1<<5)){
		
		/* We have not woken a task at the start of the ISR. */
		xHigherPriorityTaskWoken = pdFALSE;
		cIn = USART1->RDR;
		xQueueSendFromISR( osUsartTab[0].rx, &cIn, &xHigherPriorityTaskWoken );

		/* Now the buffer is empty we can switch context if necessary. */
		if( xHigherPriorityTaskWoken )
		{
				/* Actual macro used here is port specific. */
				taskYIELD();
		}
	}
	USART1->ISR = 0;
	//HAL_NVIC_ClearPendingIRQ(USART1_IRQn );
}
/**
  * @brief  Funkcja przerwania od UARTA2
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	char cIn;

	if(USART2->ISR&(0x1<<5)){
		
		/* We have not woken a task at the start of the ISR. */
		xHigherPriorityTaskWoken = pdFALSE;
		cIn = USART2->RDR;
		xQueueSendFromISR( osUsartTab[1].rx, &cIn, &xHigherPriorityTaskWoken );

		/* Now the buffer is empty we can switch context if necessary. */
		if( xHigherPriorityTaskWoken )
		{
				/* Actual macro used here is port specific. */
				taskYIELD();
		}
	}
	USART2->ISR = 0;
}

