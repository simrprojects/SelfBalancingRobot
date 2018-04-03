/**
  ******************************************************************************
  * @file	 CAN2UART.c
  * @author  Przemek
  * @version V1.0.0
  * @date    02.04.2018
  * @brief   Modu³ dostarcza funkcji wirtualnego portu UART zbudowanego w oparciu o komunikacjê na UART
  * 		 Modu³ rezerwuje do pracy jeden odbiorczy FIFO od CAN. Modu³ tworzy abstrakcjê kana³ów UART.
  * 		 Z ka¿dym kana³em zwi¹zany jest identyfikator ramki nadawzej oraz identyfikator ramki odbiorczej
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "OSCan.h"
#include "CAN2UART.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	int rxID;
	int txID;
	xQueueHandle rxQueue;
	xQueueHandle txQueue;
	xTaskHandle task;
}tCAN2UARTChannel;

typedef struct{
	tCAN2UARTChannel *channelArray;
	int maxNumberOfChannels;
	int channelCnt;
	int canFifoNumber;
	xTaskHandle task;
}tCAN2UART;
/* Private define ------------------------------------------------------------*/
#define CAN_CH_REF()	((tCAN2UARTChannel*)h)
#define CAN_H()			((tCAN2UART*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CAN2UART_RxTask(tCAN2UARTHandle h);
void CAN2UART_TxTask(tCAN2UARTChannelRef h);
void CAN2UART_Error(char* errorDescription);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje modu³, tworzy w¹tek odbiorczy odbieraj¹cy ramki.
  * @param[in]  None
  * @retval None
  */
int CAN2UART_Init(tCAN2UARTHandle *h,tCAN2UARTConfig *cfg){
	//alokuje pamiêc na modul
	tCAN2UART *c2u;
	c2u = pvPortMalloc(sizeof(tCAN2UART));
	//inicjuje zmienna
	c2u->channelCnt=0;
	c2u->maxNumberOfChannels=cfg->maxNumberOfChannels;
	c2u->canFifoNumber = cfg->canFifoNumber;
	c2u->channelArray = pvPortMalloc(sizeof(tCAN2UARTChannel)*cfg->maxNumberOfChannels);
	//inicjuje w¹tek
	xTaskCreate(CAN2UART_RxTask,"can2uart",256,c2u,4,&c2u->task);
	*h = c2u;
	return 0;
}
/**
  * @brief  Funkcja inicjuje kana³ komunikacyjny. Z kana³em zwi¹zany jest w¹tek nadawczy, pakuj¹cy dane do ramki CAN i wysy³aj¹cy je do CAN
  *         oraz rejestracja kolejki odbiorczej do przenoszenia danych odebranych na CAN a zwi¹zanych z danym kana³em identyfikatorem ramki CAN
  * @note	Ka¿dy kana³ inicjuje jeden blok sprzetowych filtrów CAN, wiêc mo¿na powo³ac tylko okreœlon¹ ich liczbê
  * @param[in] h: referencja do obiektu modu³u
  * @param[out] chRef: referencja do uchwytu do kana³u
  * @param[in] cfg: wskaxnik do obiektu opisuj¹cego konfiguracjê kana³u
  * @retval 0 - brak b³êdów
  */
int CAN2UART_CreateChannel(tCAN2UARTHandle h,tCAN2UARTChannelRef *chRef,tCAN2UARTChannelCfg *cfg){
	tCAN2UARTChannel *ch;
	tCanFilter f;
	int fid;
	//sprawdzam, czy mam wolne miejsce na kana³
	if(CAN_H()->channelCnt<CAN_H()->maxNumberOfChannels){
		//jest wolne miejsce
		ch = &CAN_H()->channelArray[CAN_H()->channelCnt++];
		*chRef = ch;
		//inicjuje filtr
		f.FIFOSelect = CAN_H()->canFifoNumber;
		f.enable=1;
		f.mode = eFilterListMode;
		f.scale = eStdFilterSize;
		f.filter[0].id.std=cfg->rxID;
		f.filter[0].ide=0;
		f.filter[0].rtr=0;
		f.filter[1].id.std=cfg->rxID;
		f.filter[1].ide=0;
		f.filter[1].rtr=0;
		f.mask[0].id.std=cfg->rxID;
		f.mask[0].ide=0;
		f.mask[0].rtr=0;
		f.mask[1].id.std=cfg->rxID;
		f.mask[1].ide=0;
		f.mask[1].rtr=0;
		if(OSCan_FilterInit(&f,&fid)){
			CAN2UART_Error("CAN2UART: B³ad inicjacji filtra warstwy CAN");
			return 2;
		}
		//ustawiam parametry
		ch->rxID = cfg->rxID;
		ch->txID = cfg->txID;
		//alokuje kolejki
		ch->rxQueue = xQueueCreate(cfg->rxQueueDepth,1);
		ch->txQueue = xQueueCreate(cfg->txQueueDepth,1);
		//tworze w¹tek nadawczy
		xTaskCreate(CAN2UART_TxTask,"txCan2Uart",256,ch,4,&ch->task);
		return 0;
	}else{
		CAN2UART_Error("CAN2UART: Brak miejsca na kolejny kana³");
		return 1;
	}
	return -1;
}
/**
  * @brief  Funkcja odczytuje jeden znak z kolejki odbiorczej kana³u
  * @param[in] chRef: referencja do kana³u
  * @param[out] data: referencja do znaku
  * @param[in] timeut: maksymalny czas oczekiwania na znak
  * @retval 0 - brak b³edu
  */
int CAN2UART_Receive(tCAN2UARTChannelRef h,char* data,int timeout){
	if(xQueueReceive(CAN_CH_REF()->rxQueue,data,timeout)==pdTRUE){
		return 0;
	}else{
		return 1;
	}
}
/**
  * @brief  Funkcja ³aduje dane do kolejki nadawczej kana³u
  * @param[in] chRef: referencja do kana³u
  * @param[in] data: referencja do danych
  * @param[in] dataSize: rozmiar danych
  * @param[in] timeut: maksymalny czas oczekiwania na wys³anie
  * @retval 0 - brak b³edu
  */
int CAN2UART_Transmit(tCAN2UARTChannelRef h,void* data,int dataSize,int timeout){
	char *tx=data;
	for(int i=0;i<dataSize;i++){
		if(xQueueSend(CAN_CH_REF()->txQueue,&tx[i],timeout)!=pdTRUE){
			//problem z wys³aniem znaku
			CAN2UART_Error("CAN2UART: Przepe³niono bufor nadawczy");
			return 1;
		}
	}
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja w¹tku odbiorczego. Funkcja odczytuje dane z CAN, a nastêpnie umieszcza je w odpowiedniej kolejce zwi¹zanej z kana³em, o odpowiednim ID
  * @param[in] h: referencja do obiektu modu³u
  * @retval None
  */
void CAN2UART_RxTask(tCAN2UARTHandle h){
	tCAN2UART *c2u = (tCAN2UART*)h;
	tCanRxMessage rx;
	tCAN2UARTChannel *ch;
	while(1){
		//odczytuje dane z odpowiedniego fifo
		if(OSCan_ReceiveMessage(&rx,c2u->canFifoNumber,100)==0){
			//przeszukuje kana³y w poszukiwaniu odpowiedniego id
			for(int i=0;i<c2u->channelCnt;i++){
				ch = &c2u->channelArray[i];
				if(rx.id==ch->rxID){
					//znalaz³em odpowiednie ID, przekazuje dane
					for(int j=0;j<rx.dlc;j++){
						xQueueSend(ch->rxQueue,&rx.data[j],100);
					}
					break;
				}
			}
		}
	}
}
/**
  * @brief  Wielokrotna instacja w¹tku odpowiedzialnego za odbieranie danych z kolejki nadawczej kana³u i ³adowanie ich do ramki CAN
  * @param[in]  @param[in] h: referencja do obiektu modu³u
  * @retval None
  */
void CAN2UART_TxTask(tCAN2UARTChannelRef h){
	tCAN2UARTChannel *ch=(tCAN2UARTChannel*)h;
	int timeout = 100;
	int id=0;
	char c;
	tCanTxMessage tx;
	//konfigurujê ramkê
	tx.id = ch->txID;
	tx.ide=0;
	tx.rtr=0;

	while(1){
		//odczytuje dane z kolejki
		if(xQueueReceive(ch->txQueue,&c,timeout)==pdTRUE){
			//odebra³em, umieszczam dane w ramce
			tx.data[id++]=c;
			if(id>=8){
				//skompletowa³em ramkê, mogê j¹ wysy³ac
				tx.dlc=8;
				OSCan_SendMessage(&tx,50);
				id = 0;
				timeout = 100;
			}else{
				//ramka nieskompletowana, ustawiam krótki czas oczekiwania
				timeout = 2;
			}
		}else{
			//timeout przekrocozny, sprawdzam, czy s¹ jakieœ dane do wys³ania
			if(id>0){
				//s¹ jakieœ dane do wys³lania
				tx.dlc = id;
				OSCan_SendMessage(&tx,50);
				id=0;
				timeout = 100;
			}
		}
	}
}

