/**
  ******************************************************************************
  * @file	 MotorInterface.c
  * @author  Przemek
  * @version V1.0.0
  * @date    02.04.2018
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MotorInterface.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	xTaskHandle task;
	tCAN2UARTChannelRef com;
	tMotorInterfaceMode mode;
	float voltage;
	float current;
	float rpm;
	float angle;
	tMotorMeasuremenets measurements;
	unsigned short currentRef;/*< warto�c srednia pradu z pomiaru gdy prad nie p�ynie*/
	unsigned short angleOffset;
	struct{
		int state;
		int size;
		int cnt;
		union{
			char id;
			char data[40];
		}dataFrame;
	}parser;
}tMotorInterface;
/* Private define ------------------------------------------------------------*/
#define MIH()	((tMotorInterface*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void MotorInterface_Task(void* ptr);
void MotorInterface_SendMessage(tCAN2UARTChannelRef *ch,char id,void *data,int dataSize);
int  MotorInterface_Parse(tMotorInterface *mi,char c);
void MotorInterface_ParseNewMeasurement(tMotorInterface *mi,char* buffer,int size);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicuje jedn� instancj� silnika, z kt�rym komunikacja odbywa si� poprzez magistral� UART/CAN
  * Funkcja powoluje jeden watek, kt�ry zajmuje si� przetwarzaniem informacji z sieci komunikacyjnej
  * @param[out] h: referencja do obiektu
  * @param[in] cfg: strukt�ra konfiguracyjna
  * @retval 0 - brak b��d�w
  */
int MotorInterface_Init(tMotorInterfaceHandler *h,tMotorInterfaceConfig *cfg){
	tMotorInterface *mi;
	tCAN2UARTChannelCfg ucfg;
	//alokuj� pami�c
	mi=pvPortMalloc(sizeof(tMotorInterface));
	//tworze kana� komunikacyjny
	ucfg.rxID = cfg->canId;
	ucfg.txID = cfg->canId<<8;
	ucfg.rxQueueDepth=64;
	ucfg.txQueueDepth=64;
	CAN2UART_CreateChannel(cfg->c2u,&mi->com,&ucfg);
	//inicjuje zmienne wewn�trzne
	mi->mode = eInactiveMode;
	mi->angleOffset=0;
	mi->currentRef=2100;
	mi->mode = eInactiveMode;
	//tworze w�tek kontrolny
	xTaskCreate(MotorInterface_Task,"motorInterface",256,mi,4,&mi->task);
	*h=mi;
	return 0;
}
/**
  * @brief  Funkcja wysy�a �adanie ustawienia trybu pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[in]  mode: tryb pracy kontrolera
  * @retval 0 - brak b��d�w
  */
int MotorInterface_SetMode(tMotorInterfaceHandler h,tMotorInterfaceMode mode){

	if(mode!=MIH()->mode){
		MotorInterface_SendMessage(MIH()->com,1,&mode,1);
	}
	return 0;
}
/**
  * @brief  Funkcja ustawia g��wny tryb pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[in]  mode: tryb pracy kontrolera
  * @retval 0 - brak b��d�w
  */
int MotorInterface_UpdateControl(tMotorInterfaceHandler h,int ctrl){
	signed short uctrl = ctrl;
	if(MIH()->mode==eActiveMode){
		MotorInterface_SendMessage(MIH()->com,4,&uctrl,2);
	}
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c pr�dkosci odczytanej z silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  rpm: wska�nik do zmiennej wyra�aj�cej pr�dkosci w obr/min
  * @retval 0 - brak b��d�w
  */
int MotorInterface_GetSpeed(tMotorInterfaceHandler h,float *rpm){
	*rpm = MIH()->measurements.rpm;
	return 0;
}
/**
  * @brief  Funkcja zwraca pozycje kontow� silnika oszacowan� na podstawie czujnik�w po�o�enia sinika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  angle: k�t wyrazony w stopniach
  * @retval 0 - brak b��d�w
  */
int MotorInterface_GetPosition(tMotorInterfaceHandler h,float *angle){
	*angle=MIH()->measurements.angle;
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c pr�du silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  current: pr�d fazowy wyra�ony w amperach
  * @retval 0 - brak b��d�w
  */
int MotorInterface_GetCurrent(tMotorInterfaceHandler h,float *current){
	*current = MIH()->measurements.current;
	return 0;
}
/**
  * @brief  Funkcja zwraca warto�c napi�cia na zaciskach akumulatora
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  voltage: napi�cie wyra�one w voltach
  * @retval 0 - brak b��d�w
  */
int MotorInterface_GetVoltage(tMotorInterfaceHandler h,float *voltage){
	*voltage = MIH()->measurements.voltage;
	return 0;
}
/**
  * @brief  Funkcja zwraca informacje o trybie pracy silnika
  * @param[in]  h: referencja do obiektu slnika
  * @param[out]  mode: aktualny tryb pracy sterownika
  * @retval 0 - brak b��d�w
  */
int MotorInterface_GetState(tMotorInterfaceHandler h,tMotorInterfaceMode *mode){
	*mode = MIH()->mode;
	return 0;
}
/**
  * @brief  Funkcja zwraca strukturę ze wszystkimi podstawowymi pomiarami silnika
  * @param[in]  None
  * @retval None
  */
tMotorMeasuremenets* MotorInterface_GetMeasurements(tMotorInterfaceHandler h){
	return &MIH()->measurements;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  W�tek przetwarza dane przychodz�ce od silnika
  * @param[in]  None
  * @retval None
  */
void MotorInterface_Task(void* ptr){
	tMotorInterface *mi = (tMotorInterface*)ptr;
	char c;
	tMotorInterfaceMode newMode;
	mi->parser.state=0;
	while(1){
		if(CAN2UART_Receive(mi->com,&c,100)==0){
			//odebra�em znak, parsuj� ramk�
			if(MotorInterface_Parse(mi,c)){
				//skompletowa�em ramk�, dokonuj� jej analizy
				switch(mi->parser.dataFrame.id){
				case 0:
					break;
				case 1:
					newMode = (tMotorInterfaceMode)mi->parser.dataFrame.data[1];
					if(newMode!=mi->mode){
						//nowy stan pracy silnika
						mi->mode = newMode;
						MotorInterface_NewMotorState(mi,newMode);
					}
					break;
				case 2://nowe pomiary
					MotorInterface_ParseNewMeasurement(mi,&mi->parser.dataFrame.data[1],mi->parser.size-2);
					break;
				case 3:
					break;
				}
			}
		}else{
			//timeout, resetuje stan maszyny stnowej
			mi->parser.state=0;
		}
	}
}
/**
  * @brief  Funkcja formatuje i wysy�a ramk� do sterownika silnika
  * @param[in]  None
  * @retval None
  */
void MotorInterface_SendMessage(tCAN2UARTChannelRef *ch,char id,void *data,int dataSize){
	unsigned char buff[4];
	buff[0]=0x12;
	buff[1]=0x34;
	buff[2]=dataSize+2;
	buff[3]=id;
	CAN2UART_Transmit(ch,buff,4,30);
	CAN2UART_Transmit(ch,data,dataSize,40);
	buff[0]=0x56;
	CAN2UART_Transmit(ch,buff,1,30);
}
/**
  * @brief  Funkcja parsuje ramke
  * @param[in]  None
  * @retval None
  */
int  MotorInterface_Parse(tMotorInterface *mi,char c){
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
void MotorInterface_ParseNewMeasurement(tMotorInterface *mi,char* buffer,int size){
	unsigned short tmp;
	if(size>=8){
		tmp = *(unsigned short*)&buffer[2];
		mi->measurements.voltage = (float)tmp*0.01543;/*wsp�czynnik skaluj�cy wynika z podzia�u dzielnika=0.052212,
																		   rozdizelczo�ci przetwornika=12bit,
																		   napi�cia referencyjnego=3.3V*/
		tmp = *(unsigned short*)&buffer[0];
		mi->measurements.current = ((int)tmp - (int)mi->currentRef)*0.0105;/* Wsp�lczynnik skaluj�cy pradu*/
		mi->measurements.rpm = *(signed short*)&buffer[4];
		tmp = *(unsigned short*)&buffer[6];
		mi->measurements.angle = (int)(tmp - mi->angleOffset)*360/(6*15);
	}
}
/**
  * @brief  Funkcja zg�asza fakt wykrycia zmiany stanu silnika
  * @param[in]  None
  * @retval None
  */
__weak void MotorInterface_NewMotorState(tMotorInterfaceHandler h,tMotorInterfaceMode mode){

}
