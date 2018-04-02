/**
  ******************************************************************************
  * @file		 OSCan.c
  * @author  PS
  * @version V1.0.0
  * @date    22.02.2017
  * @brief   Moduł dostarcza podstawowych funkcji do obsłógi interfejsu CAN w oparciu o 
	*					 warstwę obsłógi sprzętu HAL oraz system operacyjny FreeRTOS
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "can.h"
#include "OSCan.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	xQueueHandle canRx0Queue;
	xQueueHandle canRx1Queue;
	xQueueHandle canTxQueue;
	xQueueHandle txMutex;
	xSemaphoreHandle txSemaphore;
	struct{
		unsigned int TSR;
		unsigned int ESR;
	}lastTxStatus;
}tOSCan;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef * phcan;
CAN_TypeDef * CAN;
tOSCan *osCan;
/* Private function prototypes -----------------------------------------------*/
int CAN_Transmit(CAN_HandleTypeDef *hcan,tCanTxMessage *tx);
/* Public  functions ---------------------------------------------------------*/
int OSCan_Init(CAN_HandleTypeDef *can,tCanInit *cfg){
	tOSCan *pCan;
	static CanTxMsgTypeDef TxMessage;
  static CanRxMsgTypeDef RxMessage;
	phcan = can;
	CAN = can->Instance;
	phcan->pTxMsg = &TxMessage;
  phcan->pRxMsg = &RxMessage;
	pCan=pvPortMalloc(sizeof(tOSCan));
	pCan->canRx0Queue = xQueueCreate(cfg->rxBufferSize,sizeof(tCanRxMessage));
	pCan->canRx1Queue = xQueueCreate(cfg->rxBufferSize,sizeof(tCanRxMessage));
	//tworze mutex zarzadzajacy dostepem do funkcji nadaczej
	pCan->txMutex = xSemaphoreCreateMutex();
	//tworze semafor do blokowania funkcji oczekujacej na koniec transmisji
	vSemaphoreCreateBinary( pCan->txSemaphore );
	//blokuje od razu semafor
	xSemaphoreTake( pCan->txSemaphore, 0);
	//uaktualniam stan
	//ustawiam podział banku filtrów pół na poł
	CAN->FMR |= 0x1;
	CAN->FMR |= (14<<8);
	CAN->FMR &= ~0x1;
	osCan = pCan;
	//uruchamiam procedury odbiorcze
	//HAL_CAN_Receive_IT(can, CAN_FIFO0);
	//HAL_CAN_Receive_IT(can, CAN_FIFO1);
	CAN->IER |= 0x7E;
	return 0;
}
 /**
   * @brief  Funkcja inicjuje filtr dla CAN
	 * Funkcja wyszukuje wolnego miejsca w strukturze filtru i jesli je znajdzie zajmuje to miejsce i inicjuje filtr
	 * Jesli funkcja nie znajdzie miejsca, zwraca blad, w przypadku pomyslnej inicjacji zwraca 0
	 * Struktura filter, zawiera wewnatrz dwie dwuelementowe tablice konfiguracji filtra. Ich uzycie zalezy od wyboru
	 * trybu pracy filtra List/Mask oraz rozmiaru identyfikatora 11b/29b
	 * zaleznie od konfiguracji filtra nalezy stosowac odpowiednie tablice filter[] oraz mask[]
	 * LIST+11b
	 * filter[0],filter[1],mask[0],mask[1]
	 *
	 * LIST+29b
	 * filter[0],filter[1]
	 *
	 * MASK+11b
	 * filter[0],mask[0],filter[1],mask[1]
	 *
	 * MASK+29b
	 * filter[0],mask[0]
   * @param[in]  filter: wskaźnik ze struktrą konfiguracyjną filtru
	 * @param[out] filterId: wskaźnik do zmiennej przechowującej indeks filtru
   * @retval 	0 - ok
	 * 					1 - brak miejsca w strukturze filtrów
	 * 					2 - nie zainicjowany moduł CAN
   */
int OSCan_FilterInit(tCanFilter *filter,int* filterId){
	CAN_FilterConfTypeDef CAN_FilterInitStructure;
	int i;
	*filterId=-1;
	//wyszukuje wolnego miejsca na filtr
	for(i=0;i<14;i++){
		if(CAN->FA1R&(0x1<<i)){
			//filtr aktywowany, szukam dalej
		}else{
			//filtr nieaktywny
			//zapisuje identyfikator
			*filterId=i;
			break;
		}
	}
	if(*filterId==-1){
		return 1;
	}
	//inicjuje strukture
	CAN_FilterInitStructure.FilterNumber = *filterId;
	CAN_FilterInitStructure.FilterMode = filter->mode;
	CAN_FilterInitStructure.FilterScale = filter->scale;
	switch(filter->mode){
		case eFilterMaskMode:
			switch(filter->scale){
				case eStdFilterSize:
					CAN_FilterInitStructure.FilterIdHigh = ((filter->mask[0].ide&(0x1))<<3) | ((filter->mask[0].rtr&(0x1))<<4) | ((filter->mask[0].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterIdLow = ((filter->filter[0].ide&(0x1))<<3) | ((filter->filter[0].rtr&(0x1))<<4) | ((filter->filter[0].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterMaskIdHigh = ((filter->mask[1].ide&(0x1))<<3) | ((filter->mask[1].rtr&(0x1))<<4) | ((filter->mask[1].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterMaskIdLow = ((filter->filter[1].ide&(0x1))<<3) | ((filter->filter[1].rtr&(0x1))<<4) | ((filter->filter[1].id.std&(0x7FF))<<5);
					break;
				case eExtFilterSize:
					CAN_FilterInitStructure.FilterIdHigh =((filter->filter[0].id.ext&(0x1FFFFFFF))>>13);
					CAN_FilterInitStructure.FilterIdLow = ((filter->filter[0].ide&(0x1))<<1) | ((filter->filter[0].rtr&(0x1))<<2) | ((filter->filter[0].id.ext&(0x1FFF))<<3);
					CAN_FilterInitStructure.FilterMaskIdHigh = ((filter->mask[0].id.ext&(0x1FFFFFFF))>>13);
					CAN_FilterInitStructure.FilterMaskIdLow = ((filter->mask[0].ide&(0x1))<<1) | ((filter->mask[0].rtr&(0x1))<<2) | ((filter->mask[0].id.ext&(0x1FFF))<<3);
					break;
			}
			break;
		case eFilterListMode:
			switch(filter->scale){
				case eStdFilterSize:
					CAN_FilterInitStructure.FilterIdHigh = ((filter->filter[0].ide&(0x1))<<3) | ((filter->filter[0].rtr&(0x1))<<4) | ((filter->filter[0].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterIdLow = ((filter->filter[1].ide&(0x1))<<3) | ((filter->filter[1].rtr&(0x1))<<4) | ((filter->filter[1].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterMaskIdHigh = ((filter->mask[0].ide&(0x1))<<3) | ((filter->mask[0].rtr&(0x1))<<4) | ((filter->mask[0].id.std&(0x7FF))<<5);
					CAN_FilterInitStructure.FilterMaskIdLow = ((filter->mask[1].ide&(0x1))<<3) | ((filter->mask[1].rtr&(0x1))<<4) | ((filter->mask[1].id.std&(0x7FF))<<5);
					break;
				case eExtFilterSize:
					CAN_FilterInitStructure.FilterIdHigh =((filter->filter[0].id.ext&(0x1FFFFFFF))>>13);
					CAN_FilterInitStructure.FilterIdLow = ((filter->filter[0].ide&(0x1))<<1) | ((filter->filter[0].rtr&(0x1))<<2) | ((filter->filter[0].id.ext&(0x1FFF))<<3);
					CAN_FilterInitStructure.FilterMaskIdHigh = ((filter->filter[1].id.ext&(0x1FFFFFFF))>>13);
					CAN_FilterInitStructure.FilterMaskIdLow = ((filter->filter[1].ide&(0x1))<<1) | ((filter->filter[1].rtr&(0x1))<<2) | ((filter->filter[1].id.ext&(0x1FFF))<<3);
					break;
			}
			break;
	}
	//przydzielam filtr do odpowiedniego bufora
	if(filter->FIFOSelect){
		CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FilterFIFO1;
	}else{
		CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	}
	CAN_FilterInitStructure.FilterActivation = (FunctionalState)filter->enable;
	HAL_CAN_ConfigFilter(phcan,&CAN_FilterInitStructure);
	return 0;
}
/**\brief Funkcja odczytuje wiadomość z kolejki
 * Zwraca:
 * 0 - ok;
 * 1 - brak wiadomoݣi
 * 2 - modul nieaktywny 
 */
int OSCan_ReceiveMessage(tCanRxMessage *rx,int bufferId,int timeout){
	if(bufferId){
		if( xQueueReceive( osCan->canRx1Queue, rx, ( portTickType ) timeout )==pdTRUE ){
			return 0;
		}else{
			return 1;
		}
	}else{
		if( xQueueReceive( osCan->canRx0Queue, rx, ( portTickType ) timeout )==pdTRUE ){
			return 0;
		}else{
			return 1;
		}
	}
}
/**\brief Funkcja wysyԡ wiadomoݦ poprzez interfejs CAN
 * Zwraca:
 * 0 - ok;
 * 1 - zapelniona kolejka
 * 2 - modul nieaktywny, wymaga inicjacji 
 * 3 - timeout
 * 4 - nieznany problem z wyslaniem ramki
 * 5 - utrata arbitrażu, ramka nie wysłana
 * 6 - błąd w trakcie wysyłania ramki, na bitach[11:3] zmapowano
 *     status błędu z rejestru ESR
 * 7 - inny nieznany błąd
 */
int OSCan_SendMessage(tCanTxMessage *tx,int timeout){
	int txMbx,status,ret;
	//pobieram mutexa
	if( xSemaphoreTake( osCan->txMutex, timeout) == pdTRUE ){
			//uzyskano dostep do procedury
			//kopjuje ramkę
//			memcpy(phcan->pTxMsg->Data,tx->data,tx->dlc);
//			phcan->pTxMsg->DLC=tx->dlc;
//			phcan->pTxMsg->IDE=tx->ide;
//			phcan->pTxMsg->RTR=tx->rtr;
//			if(tx->ide){
//				phcan->pTxMsg->ExtId = tx->id;
//			}else{
//				phcan->pTxMsg->StdId = tx->id;
//			}
			//wysylam ramke
			//HAL_CAN_Transmit_IT(phcan);
			//txMbx = phcan->
			txMbx=CAN_Transmit(phcan,tx);
			//pobieram semafor i czekam na koniec transmisji
			if(xSemaphoreTake(osCan->txSemaphore,100)==pdTRUE){
				//odebralem semafor, odczytuje status
				status = osCan->lastTxStatus.TSR >>(8*txMbx);
				//sprawdzam czy pomyslnie odebrano ramke
				if(status&0x02){
					//raka pomyslnie odebrana
					//zwalniam mutex
					xSemaphoreGive( osCan->txMutex);
					return 0;
				}else if(status&0x04){
					//utrata arbitrazu
					//zwalniam mutex
					xSemaphoreGive( osCan->txMutex);
					return 5;
				}else if(status&0x08){
					//błąd komunikaci
					ret = 6 | ((osCan->lastTxStatus.ESR&0xFF)<<4);
					//zwalniam mutex
					xSemaphoreGive( osCan->txMutex);
					return ret;
				}else{
					return 7;
				}
			}else{
				//niedoczekalem sie na wyslanie ramki
				//zwalniam mutex
				xSemaphoreGive( osCan->txMutex);
				return 4;
			}
		}else{
			//niedoczekalem sie na dostep do procedury
			return 3;
		}
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Zoptymalizowana funkcja biblioteki HAL do wysyłania wiadomości
  * @param  None
  * @retval None
  */
int CAN_Transmit(CAN_HandleTypeDef *hcan,tCanTxMessage *tx){
	uint32_t  transmitmailbox = CAN_TXSTATUS_NOMAILBOX;
  

  
  if(((hcan->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0) || \
     ((hcan->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) || \
     ((hcan->Instance->TSR&CAN_TSR_TME2) == CAN_TSR_TME2))
  {
    /* Process Locked */
    //__HAL_LOCK(hcan);

    /* Select one empty transmit mailbox */
    if((hcan->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
    {
      transmitmailbox = CAN_TXMAILBOX_0;
    }
    else if((hcan->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
    {
      transmitmailbox = CAN_TXMAILBOX_1;
    }
    else
    {
      transmitmailbox = CAN_TXMAILBOX_2;
    }

    /* Set up the Id */
    hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
    if(tx->ide == CAN_ID_STD)
    {
      //assert_param(IS_CAN_STDID(hcan->pTxMsg->StdId));  
      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((tx->id << 21U) | (tx->rtr<<1));
    }
    else
    {
      //assert_param(IS_CAN_EXTID(hcan->pTxMsg->ExtId));
      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((tx->id << 3U) | (tx->ide<<2) |  (tx->rtr<<1));
    }
    
    /* Set up the DLC */
    tx->dlc &= (uint8_t)0x0000000F;
    hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t)0xFFFFFFF0U;
    hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= tx->dlc;

    /* Set up the data field */
    hcan->Instance->sTxMailBox[transmitmailbox].TDLR = *((uint32_t*)tx->data);
    hcan->Instance->sTxMailBox[transmitmailbox].TDHR = *((uint32_t*)&tx->data[4]);

    /* Change CAN state */
    switch(hcan->State)
    {
      case(HAL_CAN_STATE_BUSY_RX0):
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX0;
          break;
      case(HAL_CAN_STATE_BUSY_RX1):
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX1;
          break;
      case(HAL_CAN_STATE_BUSY_RX0_RX1):
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX0_RX1;
          break;
      default: /* HAL_CAN_STATE_READY */
          hcan->State = HAL_CAN_STATE_BUSY_TX;
          break;
    }

    /* Set CAN error code to none */
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;

    /* Process Unlocked */
    //__HAL_UNLOCK(hcan);

    /* Request transmission */
    hcan->Instance->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;

    /* Enable Error warning, Error passive, Bus-off,
       Last error and Error Interrupts */
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG |
                              CAN_IT_EPV |
                              CAN_IT_BOF |
                              CAN_IT_LEC |
                              CAN_IT_ERR |
                              CAN_IT_TME);
  }
  else
  {
    /* Change CAN state */
    hcan->State = HAL_CAN_STATE_ERROR; 

    /* Return function status */
  }
  
  return transmitmailbox;
}
void CAN_Receive(CAN_TypeDef* can, unsigned char FIFONumber,tCanRxMessage *pRxMsg)
{
  /* Get the Id */
  pRxMsg->ide = (uint8_t)0x04 & can->sFIFOMailBox[FIFONumber].RIR;
  if (pRxMsg->ide == CAN_ID_STD)
  {
    pRxMsg->id = 0x000007FFU & (can->sFIFOMailBox[FIFONumber].RIR >> 21U);
  }
  else
  {
    pRxMsg->id = 0x1FFFFFFFU & (can->sFIFOMailBox[FIFONumber].RIR >> 3U);
  }
  
  pRxMsg->rtr = (uint8_t)0x02 & can->sFIFOMailBox[FIFONumber].RIR;
  /* Get the DLC */
  pRxMsg->dlc = (uint8_t)0x0F & can->sFIFOMailBox[FIFONumber].RDTR;
  /* Get the FMI */
  //pRxMsg->FMI = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDTR >> 8U);
  /* Get the data field */
  pRxMsg->data[0] = (uint8_t)0xFF & can->sFIFOMailBox[FIFONumber].RDLR;
  pRxMsg->data[1] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDLR >> 8U);
  pRxMsg->data[2] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDLR >> 16U);
  pRxMsg->data[3] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDLR >> 24U);
  pRxMsg->data[4] = (uint8_t)0xFF & can->sFIFOMailBox[FIFONumber].RDHR;
  pRxMsg->data[5] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDHR >> 8U);
  pRxMsg->data[6] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDHR >> 16U);
  pRxMsg->data[7] = (uint8_t)0xFF & (can->sFIFOMailBox[FIFONumber].RDHR >> 24U);
  /* Release the FIFO */
  /* Release FIFO0 */
  if (FIFONumber == CAN_FIFO0)
  {
		can->RF0R|=1<<5;
  }
  /* Release FIFO1 */
  else /* FIFONumber == CAN_FIFO1 */
  {
    can->RF1R|=1<<5;
  }
}
/** Funkcje callback */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	portBASE_TYPE xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	//odczytuje status transmisji i bledu
	osCan->lastTxStatus.TSR=CAN1->TSR;
	osCan->lastTxStatus.ESR=CAN1->ESR;
	//przekazuje semafor
	xSemaphoreGiveFromISR(osCan->txSemaphore,&xHigherPriorityTaskWoken);
	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken ){
			/* Actual macro used here is port specific. */
			taskYIELD();
	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	portBASE_TYPE xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	//odczytuje status transmisji i bledu
	osCan->lastTxStatus.TSR=CAN1->TSR;
	osCan->lastTxStatus.ESR=CAN1->ESR;
	//przekazuje semafor
	xSemaphoreGiveFromISR(osCan->txSemaphore,&xHigherPriorityTaskWoken);
	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken ){
			/* Actual macro used here is port specific. */
			taskYIELD();
	}
}
/** Przerwania CAN1**/
void CAN1_RX0_IRQHandler(void){
	tCanRxMessage rx;
	portBASE_TYPE xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	do{
		//odczytanie wiadomość dopóki coś jest w buforze odbiorczym
		CAN_Receive(CAN1,0,&rx);
		//umieszczenie wiadomoݣi w kolejce
		xQueueSendFromISR( osCan->canRx0Queue, &rx, &xHigherPriorityTaskWoken );	
	}while(CAN1->RF0R&0x3);
	/* Now the buffer is empty we can switch context if necessary. */
    if( xHigherPriorityTaskWoken ){
        /* Actual macro used here is port specific. */
        taskYIELD();
    }
	//zwolnienie przerwania
	CAN1->RF0R |= 0x18;
}                                               
void CAN1_RX1_IRQHandler(void){
	tCanRxMessage rx;
	portBASE_TYPE xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	do{
		//odczytanie wiadomość dopóki coś jest w buforze odbiorczym
		CAN_Receive(CAN1,1,&rx);
		//umieszczenie wiadomoݣi w kolejce
		xQueueSendFromISR( osCan->canRx1Queue, &rx, &xHigherPriorityTaskWoken );	
	}while(CAN1->RF1R&0x3);
	/* Now the buffer is empty we can switch context if necessary. */
    if( xHigherPriorityTaskWoken ){
        /* Actual macro used here is port specific. */
        taskYIELD();
    }
	//zwolnienie przerwania
	CAN1->RF1R |= 0x18;
}  

//void CAN1_TX_IRQHandler(void){
//	portBASE_TYPE xHigherPriorityTaskWoken;
//	/* We have not woken a task at the start of the ISR. */
//	xHigherPriorityTaskWoken = pdFALSE;
//	//odczytuje status transmisji i bledu
//	osCan->lastTxStatus.TSR=CAN1->TSR;
//	osCan->lastTxStatus.ESR=CAN1->ESR;
//	//przekazuje semafor
//	xSemaphoreGiveFromISR(osCan->txSemaphore,&xHigherPriorityTaskWoken);
//	/* Now the buffer is empty we can switch context if necessary. */
//	if( xHigherPriorityTaskWoken ){
//			/* Actual macro used here is port specific. */
//			taskYIELD();
//	}
//	//zwolnienie przerwania
//	//CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//} 
/**\brief Przerwania po zmianie statusu CAN
 */
//void CAN1_SCE_IRQHandler(void){
//	portBASE_TYPE xHigherPriorityTaskWoken;
//	/* We have not woken a task at the start of the ISR. */
//	xHigherPriorityTaskWoken = pdFALSE;
//	//odczytuje status transmisji i bledu
//	osCan->lastTxStatus.TSR=CAN1->TSR;
//	osCan->lastTxStatus.ESR=CAN1->ESR;
//	//przekazuje semafor
//	xSemaphoreGiveFromISR(osCan->txSemaphore,&xHigherPriorityTaskWoken);
//	/* Now the buffer is empty we can switch context if necessary. */
//	if( xHigherPriorityTaskWoken ){
//			/* Actual macro used here is port specific. */
//			taskYIELD();
//	}
//	//zwolnienie przerwania
//	CAN_ClearITPendingBit(CAN1,CAN_IT_BOF);
//	CAN_ClearITPendingBit(CAN1,CAN_IT_LEC);
//	CAN_ClearITPendingBit(CAN1,CAN_IT_ERR);
//} 
/**
  * @brief  Funkcja testuje działanie procedur nadawczo odbiorczych
	* Funkcja wymaga uruchomienia CAN w trybie loopback
  * @param[in] can* wskaxnik do obiektu CAN od HAL
  * @retval 0 - brak błedów
  */
int OSCan_TestInLoopBackMode(CAN_HandleTypeDef *can){
	tCanInit init;
	tCanFilter f;
	int fid;
	tCanRxMessage rx;
	tCanTxMessage tx;
	
	init.canSpeed = eCanSpeed_250kb;
	init.rxBufferSize=20;
	OSCan_Init(can,&init);
	//inicjacja filtrów
	f.enable=1;
	f.filter[0].id.std = 10;
	f.filter[0].ide = 0;
	f.filter[0].rtr = 0;
	f.filter[1].id.std = 11;
	f.filter[1].ide = 0;
	f.filter[1].rtr = 0;
	f.mask[0].id.std = 12;
	f.mask[0].ide=0;
	f.mask[0].rtr=0;
	f.mask[1].id.std = 13;
	f.mask[1].ide=0;
	f.mask[1].rtr=0;
	f.mode = eFilterListMode;
	f.scale = eStdFilterSize;
	f.FIFOSelect=0;
	if(OSCan_FilterInit(&f,&fid)){
		return 1;
	}
	//tworzę wiadomość nadawczą
	tx.dlc=8;
	tx.id = 11;
	tx.ide=0;
	tx.rtr=0;
	*(int*)tx.data = 0x12345678;
	*(int*)&tx.data[4] = 0x12345678;
	if(OSCan_SendMessage(&tx,100)){
		return 2;
	}		
	if(OSCan_ReceiveMessage(&rx,0,100)){
		return 3;
	}
	return 0;
}

