/**
  ******************************************************************************
  * @file	 MotorInterfaceUart.c
  * @author  Przemek
  * @version V1.0.0
  * @date    10.12.2018
  * @brief	Moduł zarządza komunikacją ze sterownikiem silnika BLDC za pomocą interfejsu jednoprzewodowego UART
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MotorInterfaceUart.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum{eMotorDisconected,eMotorInactive,eMotorActive}tMotorInterfaceUartInternalState;

typedef struct{
	xTaskHandle task;
	xQueueHandle reqQueue;//kolejka do przekazwyania rzadań
	xQueueHandle resQueue;//kolejka do przkeazywania odpowiedzi na rzadania
	tMotorInterfaceMode mode;
	tMotorInterfaceUartInternalState internalState;
	OsUARTHandler uart;
	float voltage;
	float current;
	float rpm;
	float angle;
	tMotorMeasuremenets measurements;
	unsigned int revers:1;
	unsigned short currentRef;/*< warto�c srednia pradu z pomiaru gdy prad nie p�ynie*/
	unsigned short angleOffset;
	char reversMode:1;
	signed int motorControl;
	struct{
		int state;
		int size;
		int cnt;
		union{
			char id;
			char data[40];
		}dataFrame;
	}parser;
}tMotorInterfaceUart;
/* Private define ------------------------------------------------------------*/
#define MIH()	((tMotorInterfaceUart*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tMotorInterfaceUart* mifu[2];
int mifuCnt=0;
/* Private function prototypes -----------------------------------------------*/
void MotorInterfaceUart_Task(void* ptr);
void MotorInterfaceUart_SendMessage(OsUARTHandler uart,char id,void *data,int dataSize);
int MotorInterfaceUart_GetState(tMotorInterfaceUart *mi,int* state);
int MotorInterfaceUart_SetState(tMotorInterfaceUart *mi,int* state);
int MotorInterfaceUart_SetControl(tMotorInterfaceUart *mi,int* state);
int  MotorInterfaceUart_Parse(tMotorInterfaceUart *mi,char c);
void MotorInterfaceUart_ParseNewMeasurement(tMotorInterfaceUart *mi,char* buffer,int size);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicuje jedn� instancj� silnika, z kt�rym komunikacja odbywa si� poprzez magistral� UART/CAN
  * Funkcja powoluje jeden watek, kt�ry zajmuje si� przetwarzaniem informacji z sieci komunikacyjnej
  * @param[out] h: referencja do obiektu
  * @param[in] cfg: strukt�ra konfiguracyjna
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_Init(tMotorInterfaceUartHandler *h,tMotorInterfaceUartConfig *cfg){
	tMotorInterfaceUart *mi;
	//alokuj� pami�c
	mi=pvPortMalloc(sizeof(tMotorInterfaceUart));
	//tworze kana� komunikacyjny
	if(OsUART_Init(&mi->uart,cfg->uart)){
		return 1;
	}
	//inicjuje zmienne wewn�trzne
	mi->mode = eInactiveMode;
	mi->internalState=eMotorDisconected;
	mi->angleOffset=0;
	mi->currentRef=2100;
	mi->motorControl=0;
	mi->reversMode = cfg->reversMode;
	mi->reqQueue = xQueueCreate(1,sizeof(char));
	mi->resQueue = xQueueCreate(1,sizeof(char));
	//tworze w�tek kontrolny
	xTaskCreate(MotorInterfaceUart_Task,"motorInterface",256,mi,4,&mi->task);
	*h=mi;
	mifu[mifuCnt++]=mi;
	return 0;
}
/**
  * @brief  Funkcja wysy�a �adanie ustawienia trybu pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[in]  mode: tryb pracy kontrolera
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_SetMode(tMotorInterfaceUartHandler h,tMotorInterfaceMode mode){
	char req = mode;
	xQueueSend(MIH()->reqQueue,&req,200);
	req = -1;
	xQueueReceive(MIH()->resQueue,&req,200);
	return req;
}
/**
  * @brief  Funkcja ustawia g��wny tryb pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[in]  mode: tryb pracy kontrolera
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_UpdateControl(tMotorInterfaceUartHandler h,int ctrl){
	if(MIH()->mode==eActiveMode){
		if(MIH()->reversMode){
			MIH()->motorControl = -ctrl;
		}else{
			MIH()->motorControl = ctrl;
		}
	}
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c pr�dkosci odczytanej z silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  rpm: wska�nik do zmiennej wyra�aj�cej pr�dkosci w obr/min
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_GetSpeed(tMotorInterfaceUartHandler h,float *rpm){
	*rpm = MIH()->measurements.rpm;
	return 0;
}
/**
  * @brief  Funkcja zwraca pozycje kontow� silnika oszacowan� na podstawie czujnik�w po�o�enia sinika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  angle: k�t wyrazony w stopniach
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_GetPosition(tMotorInterfaceUartHandler h,float *angle){
	*angle=MIH()->measurements.angle;
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c pr�du silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  current: pr�d fazowy wyra�ony w amperach
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_GetCurrent(tMotorInterfaceUartHandler h,float *current){
	*current = MIH()->measurements.current;
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c napi�cia na zaciskach akumulatora
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  voltage: napi�cie wyra�one w voltach
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_GetVoltage(tMotorInterfaceUartHandler h,float *voltage){
	*voltage = MIH()->measurements.voltage;
	return 0;
}
/**
  * @brief  Funkcja zwraca informacje o trybie pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  mode: aktualny tryb pracy sterownika
  * @retval 0 - brak b��d�w
  */
int MotorInterfaceUart_GetMode(tMotorInterfaceUartHandler h,tMotorInterfaceMode *mode){
	if(MIH()->internalState>eMotorInactive){
		*mode = eActiveMode;
	}else{
		*mode = eInactiveMode;
	}
	return 0;
}
/**
  * @brief  Funkcja zwraca strukturę ze wszystkimi podstawowymi pomiarami silnika
  * @param[in]  None
  * @retval None
  */
tMotorMeasuremenets* MotorInterfaceUart_GetMeasurements(tMotorInterfaceUartHandler h){
	return &MIH()->measurements;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  W�tek przetwarza dane przychodz�ce od silnika
  * @param[in]  None
  * @retval None
  */
void MotorInterfaceUart_Task(void* ptr){
	tMotorInterfaceUart *mi = (tMotorInterfaceUart*)ptr;
	char c;
	mi->parser.state=0;
	int state,connError=0;
	long unsigned int time;
	while(1){
		switch(mi->internalState){
		case eMotorDisconected:
			vTaskDelay(100);//limituje częstotliwośc odpytywana sterownika silnika do 10Hz.
			if(MotorInterfaceUart_GetState(mi,&state)==0){
				//poprawnie odczytano stan
				switch(state){
				case 0://silnik niekatywny
					//przechodzę do stanu pracy nieaktywnego
					mi->internalState = eMotorInactive;
					break;
				case 1://silnik aktywny
					//deaktywuję silnik, i przechodzę do nieaktywnego trybu pracy
					state = 0;
					MotorInterfaceUart_SetState(mi,&state);
					if(state==0){
						mi->internalState = eMotorInactive;
					}
					break;
				}
			}
			break;
		case eMotorInactive:
			//odczytuję stan silnika
			if(MotorInterfaceUart_GetState(mi,&state)==0){
				//pomyślna komunikacja
				if(state==0){
					//jest ok.
				}else{
					//przełączam do trybu nieaktywnego
					state = 0;
					MotorInterfaceUart_SetState(mi,&state);
				}
			}else{
				//problem z komunikacją
				connError++;
				if(connError>10){
					//ustawiam tryb rozłaczenia
					mi->internalState = eMotorDisconected;
					connError=0;
				}
			}
			//odczytuje rozkazy z aplikacji
			if(xQueueReceive(mi->reqQueue,&c,0)==pdTRUE){
				//nowy rozkaz
				switch(c){
				case 0://przełącz do trybu nieaktywnego
					//zwracam po prostu ok.
					c=0;
					xQueueSend(mi->resQueue,&c,0);
					break;
				case 1://przełacz do trybu aktywnego
					//wysyłam polecenie do silnika
					state = 1;
					MotorInterfaceUart_SetState(mi,&state);
					//sprawdzam odpowiedź
					if(state==1){
						mi->internalState = eMotorActive;
						//przekazuję info
						c=0;
						xQueueSend(mi->resQueue,&c,0);
						//przekazuję info o zmianie stanu
						MotorInterfaceUart_NewMotorState(mi,eActiveMode);
						//zeruje sterowanie
						mi->motorControl=0;
						//odczytuje aktualny czas
						time = xTaskGetTickCount();
					}else{
						//jakiś problem
						c=1;
						xQueueSend(mi->resQueue,&c,0);
					}
					break;
				default:
					//nie rozpoznane polecenie
					c=2;
					xQueueSend(mi->resQueue,&c,0);
					break;
				}
			}else{
				//brak rozkazu, wprowadzam opóźnienie w celu redukcji częstości odpytywania sterownika
				vTaskDelay(10);
			}
			break;
		case eMotorActive:
			//wysyłam wysterowanie oraz odczytuję pomiary ze sterownika
			if(MotorInterfaceUart_SetControl(mi,&state)==0){
				//pomyślna komunikacja
				if(state==1){
					//jest ok.
				}else{
					//przełączam do trybu nieaktywnego
					mi->internalState = eMotorInactive;
					//przekazuję info o zmianie stanu
					MotorInterfaceUart_NewMotorState(mi,eInactiveMode);
				}
			}else{
				//problem z komunikacją
				connError++;
				if(connError>10){
					//ustawiam tryb rozłaczenia
					mi->internalState = eMotorDisconected;
					connError=0;
					//przekazuję info o zmianie stanu
					MotorInterfaceUart_NewMotorState(mi,eInactiveMode);
				}
			}
			//odczytuje rozkazy z aplikacji
			if(xQueueReceive(mi->reqQueue,&c,0)==pdTRUE){
				//nowy rozkaz
				switch(c){
				case 0://przełącz do trybu nieaktywnego
					//przełączam do trybu nieaktywnego
					mi->internalState = eMotorInactive;
					//wysyłam polecenie do silnika
					state = 0;
					MotorInterfaceUart_SetState(mi,&state);
					//przekazuję info o zmianie stanu
					MotorInterfaceUart_NewMotorState(mi,eInactiveMode);
					c=0;
					xQueueSend(mi->resQueue,&c,0);
					break;
				case 1://przełacz do trybu aktywnego
					c=0;
					xQueueSend(mi->resQueue,&c,0);
				default:
					//nie rozpoznane polecenie
					c=2;
					xQueueSend(mi->resQueue,&c,0);
					break;
				}
			}else{
			}
			vTaskDelayUntil(&time,10);
			break;
		}
	}
}
/**
  * @brief  Funkcja odczytuje stan pracy silnika
  * @param[in]  None
  * @retval None
  */
int MotorInterfaceUart_GetState(tMotorInterfaceUart *mi,int* state){
	char c;
	//wysyłam polecenie
	MotorInterfaceUart_SendMessage(mi->uart,5,0,0);
	//odczytuje odpowiedź
	mi->parser.state=0;
	OsUART_FlushRx(mi->uart);
	while(OsUART_Receive(mi->uart,&c,10)==0){
		//odebra�em znak, parsuj� ramk�
		if(MotorInterfaceUart_Parse(mi,c)){
			//skompletowa�em ramk�, dokonuj� jej analizy
			switch(mi->parser.dataFrame.id){
			case 5://ramka statusu
				*state = mi->parser.dataFrame.data[1];
				return 0;
			default:
				*state=-1;
				return 1;
			}
		}
	}
	*state=-1;
	return 2;
}
/**
  * @brief  Funkcja wysyła rzadanie ustawienia stanu i odczytuje aktualny stan
  * @param[in]  None
  * @retval None
  */
int MotorInterfaceUart_SetState(tMotorInterfaceUart *mi,int* state){
	char c;
	//wysyłam polecenie
	c = *state;
	MotorInterfaceUart_SendMessage(mi->uart,6,&c,1);
	//odczytuje odpowiedź
	mi->parser.state=0;
	OsUART_FlushRx(mi->uart);
	while(OsUART_Receive(mi->uart,&c,10)==0){
		//odebra�em znak, parsuj� ramk�
		if(MotorInterfaceUart_Parse(mi,c)){
			//skompletowa�em ramk�, dokonuj� jej analizy
			switch(mi->parser.dataFrame.id){
			case 6://ramka statusu
				*state = mi->parser.dataFrame.data[1];
				return 0;
			default:
				*state=-1;
				return 1;
			}
		}
	}
	*state=-1;
	return 2;
}
/**
  * @brief  Funkcja wysyła sterowanie do silnika a odczytuje pomiary oraz stan jego pracy
  * @param[in]  None
  * @retval None
  */
int MotorInterfaceUart_SetControl(tMotorInterfaceUart *mi,int* state){
	char c;
	signed short ctrl;
	//wysyłam polecenie
	ctrl = mi->motorControl;
	MotorInterfaceUart_SendMessage(mi->uart,7,&ctrl,2);
	//odczytuje odpowiedź
	mi->parser.state=0;
	OsUART_FlushRx(mi->uart);
	while(OsUART_Receive(mi->uart,&c,10)==0){
		//odebra�em znak, parsuj� ramk�
		if(MotorInterfaceUart_Parse(mi,c)){
			//skompletowa�em ramk�, dokonuj� jej analizy
			switch(mi->parser.dataFrame.id){
			case 7://ramka statusu
				*state = mi->parser.dataFrame.data[1];
				MotorInterfaceUart_ParseNewMeasurement(mi,&mi->parser.dataFrame.data[2],mi->parser.size-3);
				return 0;
			default:
				*state=-1;
				return 1;
			}
		}
	}
	*state=-1;
	return 2;
}
/**
  * @brief  Funkcja formatuje i wysy�a ramk� do sterownika silnika
  * @param[in]  None
  * @retval None
  */
void MotorInterfaceUart_SendMessage(OsUARTHandler uart,char id,void *data,int dataSize){
	unsigned char buff[4];
	buff[0]=0x12;
	buff[1]=0x34;
	buff[2]=dataSize+2;
	buff[3]=id;
	OsUART_Transmit(uart,buff,4,30);
	if(dataSize){
		OsUART_Transmit(uart,data,dataSize,40);
	}
	buff[0]=0x56;
	OsUART_Transmit(uart,buff,1,30);
}
/**
  * @brief  Funkcja parsuje ramke
  * @param[in]  None
  * @retval None
  */
int  MotorInterfaceUart_Parse(tMotorInterfaceUart *mi,char c){
	switch(mi->parser.state){
	case 0:
		if(c==0x12){
			mi->parser.state=1;
		}
		break;
	case 1:
		if(c==0x34){
			mi->parser.state=2;
		}else{
			mi->parser.state=0;
		}
		break;
	case 2:
		mi->parser.size = c;
		if(c>40){
			mi->parser.state=0;
		}else{
			mi->parser.state=3;
			mi->parser.cnt = 0;
		}
		break;
	case 3:
		mi->parser.dataFrame.data[mi->parser.cnt++]=c;
		if(mi->parser.cnt>=mi->parser.size){
			//skompletowa�em ramk�
			mi->parser.state=0;
			return 1;
		}else{
			return 0;
		}
	}
	return 0;
}
/**
  * @brief  Funkcja dokonuje przekodowania warto�ci odczytanych z pomiarow na warto�ci wyra�one w jednostkach fizycznych
  * @param[in]  None
  * @retval None
  */
void MotorInterfaceUart_ParseNewMeasurement(tMotorInterfaceUart *mi,char* buffer,int size){
	unsigned short tmp;
	if(size>=8){
		tmp = *(unsigned short*)&buffer[2];
		mi->measurements.voltage = (float)tmp*0.01543;/*wsp�czynnik skaluj�cy wynika z podzia�u dzielnika=0.052212,
																		   rozdizelczo�ci przetwornika=12bit,
																		   napi�cia referencyjnego=3.3V*/
		tmp = *(unsigned short*)&buffer[0];
		mi->measurements.current = ((int)tmp - (int)mi->currentRef)*0.0105;/* Wsp�lczynnik skaluj�cy pradu*/
		if(mi->reversMode){
			mi->measurements.rpm = -*(signed short*)&buffer[4];
		}else{
			mi->measurements.rpm = *(signed short*)&buffer[4];
		}
		tmp = *(unsigned short*)&buffer[6];
		mi->measurements.angle = (signed short)(tmp - mi->angleOffset);
	}
}
/**
  * @brief  Funkcja zg�asza fakt wykrycia zmiany stanu silnika
  * @param[in]  None
  * @retval None
  */
__weak void MotorInterfaceUart_NewMotorState(tMotorInterfaceUartHandler h,tMotorInterfaceMode mode){

}


